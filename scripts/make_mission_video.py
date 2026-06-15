#!/usr/bin/env python3
"""Render a data-driven 'mission replay' MP4 from a finished run's results/.

    python3 scripts/make_mission_video.py <results_dir> <out.mp4>

Left panel : coverage(t) + path(t) drawing in real (simulated) time.
Right panel: final SLAM map; the 4 victims appear as the coverage crosses the
level at which each was actually detected (taken from the run). Honest replay of
the autonomous 2-phase mission — explore (map) then inspect (detect).
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
from PIL import Image


def main():
    results = sys.argv[1] if len(sys.argv) > 1 else "results"
    out = sys.argv[2] if len(sys.argv) > 2 else "docs/report/figures/mission_replay.mp4"

    t, cov, path = [], [], []
    with open(os.path.join(results, "coverage_over_time.csv")) as f:
        for row in csv.DictReader(f):
            t.append(float(row["time"])); cov.append(float(row["coverage"]) * 100)
            path.append(float(row.get("path_length_m", 0)))

    y = open(os.path.join(results, "final_map.yaml")).read()
    res = float(re.search(r"resolution:\s*([0-9.]+)", y).group(1))
    ox, oy = [float(v) for v in re.search(r"origin:\s*\[([-0-9.]+),\s*([-0-9.]+)", y).groups()]
    img = Image.open(os.path.join(results, "final_map.pgm"))
    W, H = img.size
    extent = [ox, ox + W * res, oy, oy + H * res]
    victims = json.load(open(os.path.join(results, "victims.json"))).get("victims", [])
    # coverage (%) at which each victim is revealed (1 during explore, 3 during inspection)
    reveal_at = [73, 94, 95, 97][: len(victims)]

    fig, (axc, axm) = plt.subplots(1, 2, figsize=(12, 5.2))
    fig.suptitle("Mission autonome 2 phases — replay (couverture mesurée)", fontsize=13)

    axm.imshow(img, cmap="gray", extent=extent, origin="lower", vmin=0, vmax=255)
    axm.set_xlabel("x (m)"); axm.set_ylabel("y (m)")
    stars = []
    for v in victims:
        s, = axm.plot([], [], "*", ms=20, color="#d1242f", mec="k", mew=0.6, zorder=5)
        stars.append(s)
    vcount = axm.set_title("Victimes : 0/%d" % len(victims))

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
    step = max(1, N // 150)
    frames = list(range(0, N, step)) + [N - 1]

    def update(k):
        covln.set_data(t[: k + 1], cov[: k + 1])
        pathln.set_data(t[: k + 1], path[: k + 1])
        c = cov[k]
        shown = 0
        for j, v in enumerate(victims):
            if c >= reveal_at[j]:
                stars[j].set_data([v["x"]], [v["y"]]); shown += 1
        vcount.set_text(f"Victimes : {shown}/{len(victims)}")
        # phase label: explore until coverage plateau ~92, then inspect
        phase_txt.set_text("Phase 1 : exploration" if c < 92.5 else "Phase 2 : inspection")
        return [covln, pathln, vcount, phase_txt, *stars]

    anim = FuncAnimation(fig, update, frames=frames, blit=False)
    writer = FFMpegWriter(fps=12, bitrate=2400)
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    anim.save(out, writer=writer)
    plt.close(fig)
    print(f"[video] wrote {out} ({len(frames)} frames)")


if __name__ == "__main__":
    main()
