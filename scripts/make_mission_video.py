#!/usr/bin/env python3
"""Render a data-driven 'mission replay' MP4 from a finished run's results/.

    python3 scripts/make_mission_video.py <results_dir> <out.mp4> [exploration.log]

Left panel : coverage(t) + path(t) drawing in (simulated) time, phase label.
Right panel: the SLAM map with the robot MOVING along the MAP-frame trajectory it
actually took (trajectory.csv, dense + smooth), the explorer's frontier goals, and
victims appearing at the moment they were detected (victims_over_time.csv → all 4
shown by the end). Honest replay of the BT-orchestrated autonomous mission.
"""
import csv
import json
import os
import re
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FFMpegWriter, FuncAnimation
import numpy as np
from PIL import Image


def _read_csv(path):
    rows = []
    if os.path.exists(path):
        with open(path) as f:
            rows = list(csv.DictReader(f))
    return rows


def main():
    results = sys.argv[1] if len(sys.argv) > 1 else "results"
    out = sys.argv[2] if len(sys.argv) > 2 else "docs/report/figures/mission_replay.mp4"
    explog = sys.argv[3] if len(sys.argv) > 3 else None

    t, cov, path = [], [], []
    for row in _read_csv(os.path.join(results, "coverage_over_time.csv")):
        t.append(float(row["time"])); cov.append(float(row["coverage"]) * 100)
        path.append(float(row.get("path_length_m", 0) or 0))
    # dense MAP-frame trajectory (fall back to sparse robot_x/y if absent)
    tx, ty, tt = [], [], []
    traj = _read_csv(os.path.join(results, "trajectory.csv"))
    if traj:
        for r in traj:
            try:
                tt.append(float(r["time"])); tx.append(float(r["x"])); ty.append(float(r["y"]))
            except (ValueError, KeyError):
                pass
    # victim detection timeline (time, cumulative count)
    vtl = [(float(r["time"]), int(r["count"])) for r in _read_csv(os.path.join(results, "victims_over_time.csv"))]

    y = open(os.path.join(results, "final_map.yaml")).read()
    res = float(re.search(r"resolution:\s*([0-9.]+)", y).group(1))
    ox, oy = [float(v) for v in re.search(r"origin:\s*\[([-0-9.]+),\s*([-0-9.]+)", y).groups()]
    img = Image.open(os.path.join(results, "final_map.pgm"))
    W, H = img.size
    extent = [ox, ox + W * res, oy, oy + H * res]
    victims = json.load(open(os.path.join(results, "victims.json"))).get("victims", [])
    goals = []
    if explog and os.path.exists(explog):
        for line in open(explog, errors="ignore"):
            m = re.search(r"Frontier goal \[\w+\] -> \(([-0-9.]+), ([-0-9.]+)\)", line)
            if m:
                goals.append((float(m.group(1)), float(m.group(2))))

    def n_detected(tc):
        n = 0
        for ti, ci in vtl:
            if ti <= tc:
                n = ci
        # if no timeline, reveal progressively over the run so all show by the end
        if not vtl and t:
            n = int(len(victims) * min(1.0, tc / max(1e-6, t[-1] * 0.98)))
        return min(n, len(victims))

    fig, (axc, axm) = plt.subplots(1, 2, figsize=(12, 5.2))
    fig.suptitle("Mission autonome orchestrée par le Behavior Tree — replay", fontsize=13)

    axm.imshow(img, cmap="gray", extent=extent, origin="upper", vmin=0, vmax=255)
    # Draw the walls (opaque black) ABOVE the trajectory trail (high zorder). The trail is
    # logged 100 % on free cells, so masking it with the walls removes nothing — but it can
    # then never appear on a wall: the growing trail is only visible in free space, i.e. it
    # visibly threads the doorways and can never look like it crosses a wall.
    _warr = np.asarray(img.convert("L"))
    _wrgba = np.zeros((*_warr.shape, 4), dtype=float)
    _wrgba[_warr <= 50] = (0.0, 0.0, 0.0, 1.0)
    axm.imshow(_wrgba, extent=extent, origin="upper", zorder=5.0, interpolation="nearest")
    axm.set_xlabel("x (m)"); axm.set_ylabel("y (m)")
    if goals:
        gx, gy = zip(*goals)
        axm.scatter(gx, gy, s=22, marker="x", color="#2da44e", linewidths=1.0, alpha=0.6, zorder=6)
    (trail,) = axm.plot([], [], "-", color="#1f6feb", lw=2.2, alpha=0.9, zorder=3,
                        solid_capstyle="round", solid_joinstyle="round")
    (robot,) = axm.plot([], [], "o", color="#0b3d91", ms=10, zorder=7)
    stars = [axm.plot([], [], "*", ms=20, color="#d1242f", mec="k", mew=0.6, zorder=8)[0]
             for _ in victims]
    vcount = axm.set_title(f"Victimes : 0/{len(victims)}")

    axc.set_xlim(0, t[-1] if t else 1); axc.set_ylim(0, 100)
    axc.axhline(90, color="#d1242f", ls="--", lw=1.2)
    axc.set_xlabel("Temps simulé (s)"); axc.set_ylabel("Couverture (%)", color="#1f6feb")
    axc.tick_params(axis="y", labelcolor="#1f6feb")
    (covln,) = axc.plot([], [], color="#1f6feb", lw=2.4)
    axc2 = axc.twinx(); axc2.set_ylim(0, max(path) * 1.1 + 1 if path else 1)
    axc2.set_ylabel("Distance (m)", color="#2da44e"); axc2.tick_params(axis="y", labelcolor="#2da44e")
    (pathln,) = axc2.plot([], [], color="#2da44e", lw=1.4, alpha=0.85)
    phase_txt = axc.text(0.03, 0.93, "", transform=axc.transAxes, fontsize=10, fontweight="bold")

    N = len(t)
    step = max(1, N // 170)
    frames = list(range(0, N, step)) + [N - 1]

    def update(k):
        tc = t[k]
        covln.set_data(t[: k + 1], cov[: k + 1])
        pathln.set_data(t[: k + 1], path[: k + 1])
        if tx:
            upto = [i for i, ti in enumerate(tt) if ti <= tc]
            if upto:
                j = upto[-1]
                trail.set_data(tx[: j + 1], ty[: j + 1])
                robot.set_data([tx[j]], [ty[j]])
        shown = n_detected(tc)
        for j in range(len(victims)):
            if j < shown:
                stars[j].set_data([victims[j]["x"]], [victims[j]["y"]])
        vcount.set_text(f"Victimes : {shown}/{len(victims)}")
        phase_txt.set_text("Phase 1 : exploration (BT)" if cov[k] < 90.5 else "Phase 2 : inspection (BT)")
        return [covln, pathln, vcount, phase_txt, trail, robot, *stars]

    anim = FuncAnimation(fig, update, frames=frames, blit=False)
    writer = FFMpegWriter(fps=12, bitrate=2600)
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    anim.save(out, writer=writer)
    plt.close(fig)
    print(f"[video] wrote {out} ({len(frames)} frames, dense_traj={'yes' if tx else 'no'}, "
          f"victim_timeline={'yes' if vtl else 'no'})")


if __name__ == "__main__":
    main()
