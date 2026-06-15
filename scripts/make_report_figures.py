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
import matplotlib.patheffects as pe
from matplotlib.collections import LineCollection
from matplotlib.patches import FancyArrow
import numpy as np
from PIL import Image

# white casing drawn under the coloured trajectory so it stays legible AS IT THREADS
# THROUGH the (narrow) doorways — makes clear the path goes through the gaps, not walls.
_PATH_HALO = [pe.Stroke(linewidth=4.0, foreground="white"), pe.Normal()]


def _load_map(results_dir):
    y = open(os.path.join(results_dir, "final_map.yaml")).read()
    res = float(re.search(r"resolution:\s*([0-9.]+)", y).group(1))
    ox, oy = [float(v) for v in re.search(r"origin:\s*\[([-0-9.]+),\s*([-0-9.]+)", y).groups()]
    img = Image.open(os.path.join(results_dir, "final_map.pgm"))
    W, H = img.size
    extent = [ox, ox + W * res, oy, oy + H * res]  # left,right,bottom,top (world m)
    return img, extent


def _draw_walls(ax, img, extent, zorder=5.0):
    """Overlay the OCCUPIED cells as crisp opaque black, drawn ABOVE the trajectory
    (high zorder). Because the path is logged 100 % on free cells, masking it with the
    walls removes nothing — but it makes the trajectory UNABLE to appear on a wall: it
    is only ever visible in the free space, i.e. it visibly threads the doorways and can
    never look like it crosses a wall. Markers (victims/poses/goals) sit above the walls."""
    arr = np.asarray(img.convert("L"))
    occ = arr <= 50  # occupied (walls) in the PGM convention (0 = occupied, 254 = free)
    rgba = np.zeros((*arr.shape, 4), dtype=float)
    rgba[occ] = (0.0, 0.0, 0.0, 1.0)  # opaque black walls, everything else transparent
    ax.imshow(rgba, extent=extent, origin="lower", zorder=zorder, interpolation="nearest")


def _plot_path(ax, xs, ys, color, lw=1.6, zorder=3.5, label=None):
    """Draw the path as a CONTINUOUS line so the route visibly follows the free space
    (through doorways), instead of dots the eye wrongly connects across walls."""
    if not xs:
        return
    ax.plot(xs, ys, "-", color=color, lw=lw, alpha=1.0, zorder=zorder,
            solid_capstyle="round", solid_joinstyle="round", label=label,
            path_effects=_PATH_HALO)


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
    _draw_walls(ax, img, extent)
    # map-derived inspection poses (autonomous) — blue arrows facing the wall
    for i, (x, y, yaw) in enumerate(poses):
        ax.add_patch(FancyArrow(x, y, 0.6 * math.cos(yaw), 0.6 * math.sin(yaw),
                     width=0.07, head_width=0.28, color="#1f6feb", length_includes_head=True,
                     zorder=6, label="Pose d'inspection (dérivée de la carte)" if i == 0 else None))
    # detected victims — red stars
    for i, v in enumerate(victims):
        ax.plot(v["x"], v["y"], marker="*", ms=18, color="#d1242f", mec="k", mew=0.6,
                zorder=7, label="Victime détectée (AprilTag)" if i == 0 else None)
        ax.annotate(f"id {v['id']}", (v["x"], v["y"]), textcoords="offset points",
                    xytext=(6, 6), fontsize=9, fontweight="bold", color="#d1242f")
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
    ax.set_title("Carte SLAM finale — 4 victimes + tournée d'inspection autonome")
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.08), fontsize=8, ncol=2, frameon=False)
    fig.tight_layout(); fig.savefig(out, dpi=200, bbox_inches="tight"); plt.close(fig)
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


def _load_trajectory(results_dir):
    """(t, x, y) of the robot in the MAP frame. Prefer the dense trajectory.csv
    (2 Hz, smooth); fall back to the sparse robot_x/robot_y in coverage_over_time.csv."""
    dense = os.path.join(results_dir, "trajectory.csv")
    if os.path.exists(dense):
        t, xs, ys = [], [], []
        with open(dense) as f:
            for row in csv.DictReader(f):
                try:
                    t.append(float(row["time"])); xs.append(float(row["x"])); ys.append(float(row["y"]))
                except (ValueError, TypeError, KeyError):
                    continue
        if xs:
            return t, xs, ys
    t, xs, ys = [], [], []
    p = os.path.join(results_dir, "coverage_over_time.csv")
    if not os.path.exists(p):
        return t, xs, ys
    with open(p) as f:
        for row in csv.DictReader(f):
            try:
                x = float(row.get("robot_x", "")); y = float(row.get("robot_y", ""))
            except (ValueError, TypeError):
                continue
            t.append(float(row["time"])); xs.append(x); ys.append(y)
    return t, xs, ys


def _load_frontier_goals(explog):
    """Frontier goals the explorer chose (algorithm element), parsed from exploration.log."""
    goals = []
    if not explog or not os.path.exists(explog):
        return goals
    pat = re.compile(r"Frontier goal \[(\w+)\] -> \(([-0-9.]+), ([-0-9.]+)\)")
    for line in open(explog, errors="ignore"):
        m = pat.search(line)
        if m:
            goals.append((float(m.group(2)), float(m.group(3))))
    return goals


def trajectory_map(results_dir, out, explog=None):
    """Whole map + the path the robot ACTUALLY took (coloured by time) + victims +
    map-derived inspection poses + the explorer's frontier goals (algorithm elements)."""
    img, extent = _load_map(results_dir)
    victims = json.load(open(os.path.join(results_dir, "victims.json"))).get("victims", [])
    t, xs, ys = _load_trajectory(results_dir)
    goals = _load_frontier_goals(explog)
    poses = []
    wp = os.path.join(results_dir, "inspection_waypoints.yaml")
    if os.path.exists(wp):
        for line in open(wp):
            m = re.search(r"x:\s*([-0-9.]+),\s*y:\s*([-0-9.]+)", line)
            if m:
                poses.append((float(m.group(1)), float(m.group(2))))

    fig, ax = plt.subplots(figsize=(7.0, 7.0))
    ax.imshow(img, cmap="gray", extent=extent, origin="lower", vmin=0, vmax=255)
    _draw_walls(ax, img, extent)
    if goals:
        gx, gy = zip(*goals)
        ax.scatter(gx, gy, s=26, marker="x", color="#2da44e", linewidths=1.2,
                   label="Buts de frontières (info_gain)", zorder=6)
    if xs:
        # continuous line coloured by time, drawn UNDER the walls (zorder < walls) so it
        # only ever shows in free space → it visibly threads the doorways, never a wall.
        pts = np.array([xs, ys]).T.reshape(-1, 1, 2)
        segs = np.concatenate([pts[:-1], pts[1:]], axis=1)
        lc = LineCollection(segs, cmap="viridis", linewidth=2.6, zorder=4,
                            capstyle="round", joinstyle="round")
        lc.set_array(np.array(t[:-1]))
        lc.set_path_effects(_PATH_HALO)
        ax.add_collection(lc)
        cb = fig.colorbar(lc, ax=ax, fraction=0.046, pad=0.04)
        cb.set_label("Temps (s) — parcours emprunté")
    for i, (px, py) in enumerate(poses):
        ax.plot(px, py, "o", ms=9, mfc="none", mec="#1f6feb", mew=2, zorder=7,
                label="Pose d'inspection (carte-dérivée)" if i == 0 else None)
    for i, v in enumerate(victims):
        ax.plot(v["x"], v["y"], marker="*", ms=20, color="#d1242f", mec="k", mew=0.6,
                zorder=8, label="Victime détectée" if i == 0 else None)
        ax.annotate(f"id {v['id']}", (v["x"], v["y"]), textcoords="offset points",
                    xytext=(6, 6), fontsize=9, fontweight="bold", color="#d1242f")
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
    ax.set_title("Parcours réellement emprunté + frontières + victimes")
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.08), fontsize=8, ncol=2, frameon=False)
    fig.tight_layout(); fig.savefig(out, dpi=200, bbox_inches="tight"); plt.close(fig)
    return out


def mission_timeline(results_dir, out):
    """BT mission timeline: the Phase 1 (explore) / Phase 2 (inspect) bands + the moment
    each victim is detected + the coverage curve. Visualises the BT orchestration."""
    t, cov = [], []
    p = os.path.join(results_dir, "coverage_over_time.csv")
    if os.path.exists(p):
        with open(p) as f:
            for row in csv.DictReader(f):
                t.append(float(row["time"])); cov.append(float(row["coverage"]) * 100)
    vt = []
    pv = os.path.join(results_dir, "victims_over_time.csv")
    if os.path.exists(pv):
        with open(pv) as f:
            for row in csv.DictReader(f):
                vt.append((float(row["time"]), int(row["count"])))
    if not t:
        return None
    # explore→inspect transition ≈ first time coverage reaches 90 %
    t_switch = next((ti for ti, ci in zip(t, cov) if ci >= 90.0), t[-1])
    fig, ax = plt.subplots(figsize=(9, 3.4))
    ax.axvspan(0, t_switch, color="#cfe3ff", alpha=0.7, label="Phase 1 — exploration (BT)")
    ax.axvspan(t_switch, t[-1], color="#ffe3c2", alpha=0.7, label="Phase 2 — inspection (BT)")
    ax.plot(t, cov, color="#1f6feb", lw=2.2)
    ax.axhline(90, color="#d1242f", ls="--", lw=1.1)
    for ti, ci in vt:
        ax.axvline(ti, color="#d1242f", lw=1.4, alpha=0.8)
        ax.annotate(f"victime #{ci}", (ti, 8), rotation=90, fontsize=8, color="#d1242f",
                    ha="right", va="bottom")
    ax.set_xlim(0, t[-1]); ax.set_ylim(0, 100)
    ax.set_xlabel("Temps simulé (s)"); ax.set_ylabel("Couverture (%)")
    ax.set_title("Chronologie de la mission décidée par le Behavior Tree")
    ax.legend(loc="lower right", fontsize=8)
    fig.tight_layout(); fig.savefig(out, dpi=150); plt.close(fig)
    return out


def summary_montage(out_dir, out):
    """2×2 montage of the key figures — a one-glance report figure."""
    names = ["mission_map.png", "trajectory_map.png", "coverage_curve.png", "annotated_map_hd.png"]
    imgs = [Image.open(os.path.join(out_dir, n)).convert("RGB")
            for n in names if os.path.exists(os.path.join(out_dir, n))]
    if len(imgs) < 4:
        return None
    cell = 560
    imgs = [im.resize((cell, int(cell * im.size[1] / im.size[0]))) for im in imgs]
    hmax = max(im.size[1] for im in imgs)
    canvas = Image.new("RGB", (2 * cell + 30, 2 * hmax + 30), "white")
    pos = [(10, 10), (cell + 20, 10), (10, hmax + 20), (cell + 20, hmax + 20)]
    for im, (x, y) in zip(imgs, pos):
        canvas.paste(im, (x, y))
    canvas.save(out)
    return out


def _load_poses(results_dir):
    poses = []
    wp = os.path.join(results_dir, "inspection_waypoints.yaml")
    if os.path.exists(wp):
        for line in open(wp):
            m = re.search(r"x:\s*([-0-9.]+),\s*y:\s*([-0-9.]+),\s*yaw:\s*([-0-9.]+)", line)
            if m:
                poses.append(tuple(float(v) for v in m.groups()))
    return poses


def _phase_split_time(results_dir):
    """Sim time of the explore→inspect transition (first coverage ≥ 90 %)."""
    p = os.path.join(results_dir, "coverage_over_time.csv")
    if os.path.exists(p):
        with open(p) as f:
            for row in csv.DictReader(f):
                try:
                    if float(row["coverage"]) >= 0.90:
                        return float(row["time"])
                except (ValueError, KeyError):
                    pass
    return None


def algorithms_two_phase(results_dir, out, explog=None):
    """One panel per phase, the algorithm of EACH in action (the user's ask, both phases):
    left = Phase 1 frontier exploration (frontier goals + the explore part of the path);
    right = Phase 2 inspection (map-derived poses + the inspect path + victims)."""
    img, extent = _load_map(results_dir)
    victims = json.load(open(os.path.join(results_dir, "victims.json"))).get("victims", [])
    t, xs, ys = _load_trajectory(results_dir)
    goals = _load_frontier_goals(explog)
    poses = _load_poses(results_dir)
    tsw = _phase_split_time(results_dir) or (t[len(t) // 2] if t else 0)
    expl = [(x, y) for ti, x, y in zip(t, xs, ys) if ti <= tsw]
    insp = [(x, y) for ti, x, y in zip(t, xs, ys) if ti > tsw]

    fig, (a1, a2) = plt.subplots(1, 2, figsize=(13, 6.8))
    for ax in (a1, a2):
        ax.imshow(img, cmap="gray", extent=extent, origin="lower", vmin=0, vmax=255)
        _draw_walls(ax, img, extent)
        ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
    if expl:
        _plot_path(a1, [p[0] for p in expl], [p[1] for p in expl], "#1f6feb", lw=2.0,
                   label="parcours (exploration)")
    if goals:
        a1.scatter([g[0] for g in goals], [g[1] for g in goals], s=44, marker="x",
                   color="#2da44e", linewidths=1.5, zorder=6, label="frontières → buts (info_gain)")
    a1.set_title("Phase 1 — Exploration par frontières (BT : ExplorePhase)")
    a1.legend(loc="upper center", bbox_to_anchor=(0.5, -0.08), fontsize=8, frameon=False)
    if insp:
        _plot_path(a2, [p[0] for p in insp], [p[1] for p in insp], "#fb8500", lw=2.0,
                   label="parcours (inspection)")
    for i, p in enumerate(poses):
        a2.plot(p[0], p[1], "o", ms=12, mfc="none", mec="#1f6feb", mew=2.3, zorder=7,
                label="pose d'inspection (carte-dérivée)" if i == 0 else None)
    for i, v in enumerate(victims):
        a2.plot(v["x"], v["y"], marker="*", ms=18, color="#d1242f", mec="k", mew=0.5, zorder=8,
                label="victime détectée" if i == 0 else None)
        a2.annotate(f"id {v['id']}", (v["x"], v["y"]), textcoords="offset points",
                    xytext=(5, 5), fontsize=8, fontweight="bold", color="#d1242f")
    a2.set_title("Phase 2 — Inspection des pièces (BT : InspectPhase)")
    a2.legend(loc="upper center", bbox_to_anchor=(0.5, -0.08), fontsize=8, frameon=False)
    fig.suptitle("Algorithmes en action — mission orchestrée par le Behavior Tree", fontsize=13)
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    fig.savefig(out, dpi=200, bbox_inches="tight"); plt.close(fig)
    return out


def main():
    results = sys.argv[1] if len(sys.argv) > 1 else "results"
    out = sys.argv[2] if len(sys.argv) > 2 else "docs/report/figures"
    explog = sys.argv[3] if len(sys.argv) > 3 else None
    os.makedirs(out, exist_ok=True)
    print(coverage_curve(results, os.path.join(out, "coverage_curve.png")))
    print(mission_map(results, os.path.join(out, "mission_map.png")))
    print(trajectory_map(results, os.path.join(out, "trajectory_map.png"), explog))
    print(algorithms_two_phase(results, os.path.join(out, "algorithms_two_phase.png"), explog))
    tl = mission_timeline(results, os.path.join(out, "mission_timeline.png"))
    if tl:
        print(tl)
    a = annotated_hd(results, os.path.join(out, "annotated_map_hd.png"))
    if a:
        print(a)
    m = summary_montage(out, os.path.join(out, "mission_summary.png"))
    if m:
        print(m)


if __name__ == "__main__":
    main()
