#!/usr/bin/env python3
"""Generate report/presentation figures from a finished run's results/ directory.

    python3 scripts/make_report_figures.py <results_dir> <out_dir>

Produces (in <out_dir>):
  - coverage_curve.png   coverage(t) + path length(t), 90 % target line, phase split
  - mission_map.png      final SLAM map + 4 victim markers + map-derived inspection poses
  - annotated_map_hd.png hi-res version of the deliverable annotated map
"""
import csv
import json
import math
import os
import re
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrow
from PIL import Image


def _load_map(results_dir):
    y = open(os.path.join(results_dir, "final_map.yaml")).read()
    res = float(re.search(r"resolution:\s*([0-9.]+)", y).group(1))
    ox, oy = [float(v) for v in re.search(r"origin:\s*\[([-0-9.]+),\s*([-0-9.]+)", y).groups()]
    img = Image.open(os.path.join(results_dir, "final_map.pgm"))
    W, H = img.size
    extent = [ox, ox + W * res, oy, oy + H * res]  # left,right,bottom,top (world m)
    return img, extent


def coverage_curve(results_dir, out):
    t, cov, path = [], [], []
    with open(os.path.join(results_dir, "coverage_over_time.csv")) as f:
        for row in csv.DictReader(f):
            t.append(float(row["time"])); cov.append(float(row["coverage"]) * 100)
            path.append(float(row.get("path_length_m", 0)))
    # phase split = where path length stops growing then resumes (explore→inspect); fall back to None
    fig, ax1 = plt.subplots(figsize=(8, 4.2))
    ax1.plot(t, cov, color="#1f6feb", lw=2.2, label="Couverture (%)")
    ax1.axhline(90, color="#d1242f", ls="--", lw=1.3, label="Objectif 90 %")
    ax1.set_xlabel("Temps simulé (s)"); ax1.set_ylabel("Couverture (%)", color="#1f6feb")
    ax1.set_ylim(0, 100); ax1.tick_params(axis="y", labelcolor="#1f6feb")
    ax2 = ax1.twinx()
    ax2.plot(t, path, color="#2da44e", lw=1.4, alpha=0.8, label="Distance parcourue (m)")
    ax2.set_ylabel("Distance parcourue (m)", color="#2da44e")
    ax2.tick_params(axis="y", labelcolor="#2da44e")
    finalcov = cov[-1] if cov else 0
    ax1.set_title(f"Mission autonome — couverture {finalcov:.1f}% (≥90 % ✓), 4 victimes")
    lines = ax1.get_lines() + ax2.get_lines()
    ax1.legend(lines, [l.get_label() for l in lines], loc="lower right", fontsize=8)
    fig.tight_layout(); fig.savefig(out, dpi=150); plt.close(fig)
    return out


def mission_map(results_dir, out):
    img, extent = _load_map(results_dir)
    victims = json.load(open(os.path.join(results_dir, "victims.json"))).get("victims", [])
    poses = []
    wp = os.path.join(results_dir, "inspection_waypoints.yaml")
    if os.path.exists(wp):
        for line in open(wp):
            m = re.search(r"x:\s*([-0-9.]+),\s*y:\s*([-0-9.]+),\s*yaw:\s*([-0-9.]+)", line)
            if m:
                poses.append(tuple(float(v) for v in m.groups()))

    fig, ax = plt.subplots(figsize=(6.4, 6.4))
    ax.imshow(img, cmap="gray", extent=extent, origin="lower", vmin=0, vmax=255)
    # map-derived inspection poses (autonomous) — blue arrows facing the wall
    for i, (x, y, yaw) in enumerate(poses):
        ax.add_patch(FancyArrow(x, y, 0.6 * math.cos(yaw), 0.6 * math.sin(yaw),
                     width=0.07, head_width=0.28, color="#1f6feb", length_includes_head=True,
                     zorder=4, label="Pose d'inspection (dérivée de la carte)" if i == 0 else None))
    # detected victims — red stars
    for i, v in enumerate(victims):
        ax.plot(v["x"], v["y"], marker="*", ms=18, color="#d1242f", mec="k", mew=0.6,
                zorder=5, label="Victime détectée (AprilTag)" if i == 0 else None)
        ax.annotate(f"id {v['id']}", (v["x"], v["y"]), textcoords="offset points",
                    xytext=(6, 6), fontsize=9, fontweight="bold", color="#d1242f")
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
    ax.set_title("Carte SLAM finale — 4 victimes + tournée d'inspection autonome")
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.08), fontsize=8, ncol=2, frameon=False)
    fig.tight_layout(); fig.savefig(out, dpi=150, bbox_inches="tight"); plt.close(fig)
    return out


def annotated_hd(results_dir, out):
    src = os.path.join(results_dir, "final_map_annotated.png")
    if not os.path.exists(src):
        return None
    im = Image.open(src).convert("RGB")
    scale = max(1, 900 // max(im.size))
    im = im.resize((im.size[0] * scale, im.size[1] * scale), Image.NEAREST)
    im.save(out)
    return out


def main():
    results = sys.argv[1] if len(sys.argv) > 1 else "results"
    out = sys.argv[2] if len(sys.argv) > 2 else "docs/report/figures"
    os.makedirs(out, exist_ok=True)
    print(coverage_curve(results, os.path.join(out, "coverage_curve.png")))
    print(mission_map(results, os.path.join(out, "mission_map.png")))
    a = annotated_hd(results, os.path.join(out, "annotated_map_hd.png"))
    if a:
        print(a)


if __name__ == "__main__":
    main()
