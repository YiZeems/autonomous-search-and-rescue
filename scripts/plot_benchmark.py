#!/usr/bin/env python3
"""L17 bonus — plot the greedy-vs-information-gain exploration comparison.

Reads the per-run artefacts produced by scripts/sh/run_benchmark.sh:
    <exp_dir>/<algo>_run<n>/coverage_over_time.csv   (time, coverage, path_length_m)
    <exp_dir>/<algo>_run<n>/run_summary.json         (time_to_90_s, path_length_m, ...)
    <exp_dir>/<algo>_run<n>/run_status.json          (valid_run, timed_out — optional)

Only VALID runs feed the aggregate (mean ± std); invalid/interrupted runs (a WSL
reboot or a stuck robot) are reported but never silently averaged in. Individual
valid runs are drawn faintly behind the mean band so the spread is visible.

Produces (in <exp_dir>/plots/):
    coverage_over_time.png   — coverage(t), mean ± std band + individual runs
    summary_bars.png         — time-to-90%, path length, victims per strategy
and a markdown table <exp_dir>/summary.md (mean ± std, valid/DNF/timeout counts).

Usage:  python3 scripts/plot_benchmark.py [exp_dir=experiments]
Needs:  numpy, matplotlib (python3-numpy + python3-matplotlib, or pip).
"""
from __future__ import annotations

import csv
import json
import sys
import warnings
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")  # headless
import matplotlib.pyplot as plt  # noqa: E402

COLORS = {"greedy": "tab:blue", "info_gain": "tab:orange"}
VALID_MIN_S = 250.0  # same gate as run_benchmark.sh (reached 90% OR ran >= this)


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


def _is_valid(summary: dict | None, status: dict | None) -> bool:
    if status is not None and "valid_run" in status:
        return bool(status["valid_run"])
    if not summary:
        return False
    fc = summary.get("final_coverage")
    dur = summary.get("duration_s") or 0
    return isinstance(fc, (int, float)) and (
        bool(summary.get("success_coverage_90")) or dur >= VALID_MIN_S)


def _collect(exp_dir: Path):
    """Return {algo: {"runs": [ {t,cov,summary,valid,timed_out} ... ]}}."""
    data: dict[str, dict] = {}
    for run_dir in sorted(exp_dir.glob("*_run*")):
        if not run_dir.is_dir():
            continue
        algo = run_dir.name.rsplit("_run", 1)[0]
        d = data.setdefault(algo, {"runs": []})
        summary = status = None
        sp = run_dir / "run_summary.json"
        if sp.exists():
            try:
                summary = json.loads(sp.read_text())
            except json.JSONDecodeError:
                pass
        stp = run_dir / "run_status.json"
        if stp.exists():
            try:
                status = json.loads(stp.read_text())
            except json.JSONDecodeError:
                pass
        t = cov = None
        cov_csv = run_dir / "coverage_over_time.csv"
        if cov_csv.exists():
            t, cov = _read_coverage(cov_csv)
            if not t.size:
                t = cov = None
        d["runs"].append({
            "name": run_dir.name, "t": t, "cov": cov, "summary": summary,
            "valid": _is_valid(summary, status),
            "timed_out": bool(status.get("timed_out")) if status else None,
        })
    return data


def _valid_summaries(d):
    return [r["summary"] for r in d["runs"] if r["valid"] and r["summary"]]


def plot_coverage(data, out_png: Path):
    plt.figure(figsize=(8, 5))
    for algo, d in sorted(data.items()):
        runs = [r for r in d["runs"] if r["valid"] and r["t"] is not None]
        if not runs:
            continue
        color = COLORS.get(algo)
        t_max = max(r["t"][-1] for r in runs)
        grid = np.linspace(0, t_max, 240)
        rows = []
        for r in runs:
            y = np.interp(grid, r["t"], r["cov"]) * 100
            y[grid > r["t"][-1]] = np.nan          # don't extrapolate past run end
            rows.append(y)
            plt.plot(grid, y, color=color, alpha=0.25, lw=1)  # individual run
        stack = np.vstack(rows)
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", category=RuntimeWarning)
            mean = np.nanmean(stack, axis=0)
            std = np.nanstd(stack, axis=0)
        plt.plot(grid, mean, label=f"{algo} (n={len(runs)})", color=color, lw=2)
        plt.fill_between(grid, mean - std, mean + std, alpha=0.18, color=color)
    plt.axhline(90, ls="--", c="gray", lw=1, label="90% target")
    plt.xlabel("time (s)")
    plt.ylabel("map coverage (%)")
    plt.title("Exploration coverage over time — greedy vs information-gain")
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(out_png, dpi=120)
    plt.close()


def _mean_std(values):
    vals = [v for v in values if isinstance(v, (int, float))]
    if not vals:
        return float("nan"), float("nan")
    return float(np.mean(vals)), float(np.std(vals))


def plot_summary_bars(data, out_png: Path):
    algos = sorted(data)
    metrics = [
        ("time_to_90_s", "time to 90% (s)"),
        ("path_length_m", "path length (m)"),
        ("victims_detected", "victims found"),
    ]
    fig, axes = plt.subplots(1, len(metrics), figsize=(12, 4))
    for ax, (key, label) in zip(axes, metrics):
        means, stds = [], []
        for a in algos:
            m, s = _mean_std([x.get(key) for x in _valid_summaries(data[a])])
            means.append(m); stds.append(s)
        ax.bar(algos, means, yerr=stds, capsize=4,
               color=[COLORS.get(a, "gray") for a in algos])
        ax.set_title(label)
        ax.grid(True, axis="y", alpha=0.3)
        for i, m in enumerate(means):
            if not np.isnan(m):
                ax.text(i, m, f"{m:.1f}", ha="center", va="bottom")
    fig.suptitle("Summary metrics (mean ± std over valid runs)")
    fig.tight_layout()
    fig.savefig(out_png, dpi=120)
    plt.close(fig)


def write_table(data, out_md: Path):
    keys = ["final_coverage", "time_to_50_s", "time_to_75_s", "time_to_90_s",
            "path_length_m", "victims_detected", "duration_s"]
    lines = ["# Exploration benchmark — greedy vs information-gain", "",
             "Mean ± std over **valid** runs per strategy "
             f"(valid = reached 90% or ran ≥ {VALID_MIN_S:.0f} s).", "",
             "| strategy | valid | DNF (no 90%) | mean±std " + " | mean±std ".join(keys) + " |",
             "|" + "---|" * (len(keys) + 3)]
    for algo in sorted(data):
        runs = data[algo]["runs"]
        sv = _valid_summaries(data[algo])
        n_valid = len(sv)
        n_total = len(runs)
        n_dnf = sum(1 for s in sv if s.get("time_to_90_s") is None)
        cells = []
        for k in keys:
            m, s = _mean_std([x.get(k) for x in sv])
            cells.append("—" if np.isnan(m) else f"{m:.2f}±{s:.2f}")
        lines.append(f"| {algo} | {n_valid}/{n_total} | {n_dnf} | " + " | ".join(cells) + " |")
    # explicit accounting so a reboot-truncated campaign is obvious
    lines += ["", "## Run accounting", ""]
    for algo in sorted(data):
        runs = data[algo]["runs"]
        n_valid = sum(1 for r in runs if r["valid"])
        n_to = sum(1 for r in runs if r["timed_out"])
        sv = _valid_summaries(data[algo])
        sr90 = (sum(1 for s in sv if s.get("time_to_90_s") is not None) / len(sv)
                if sv else float("nan"))
        bad = [r["name"] for r in runs if not r["valid"]]
        lines.append(f"- **{algo}**: {n_valid}/{len(runs)} valid, {n_to} hit timeout, "
                     f"success_rate_90={sr90:.0%}" + (f" — invalid: {', '.join(bad)}" if bad else ""))
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
        runs = data[algo]["runs"]
        sv = _valid_summaries(data[algo])
        m90, s90 = _mean_std([s.get("time_to_90_s") for s in sv])
        mpath, _ = _mean_std([s.get("path_length_m") for s in sv])
        print(f"  {algo}: {len(sv)}/{len(runs)} valid run(s), "
              f"mean time_to_90={m90:.1f}±{s90:.1f}s, mean path={mpath:.1f}m")


if __name__ == "__main__":
    main()
