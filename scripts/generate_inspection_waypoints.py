#!/usr/bin/env python3
"""CLI: write AUTONOMOUS room-inspection waypoints from a run's results/ directory.

    python3 scripts/generate_inspection_waypoints.py <results_dir> [out.yaml]
    # reads <results_dir>/final_map.{yaml,pgm}, writes <results_dir>/inspection_waypoints.yaml

The planning logic lives in the package (rescue_robot.navigation.inspection_planner) so
the BT-orchestrated inspection_node and this CLI share ONE implementation. No victim
coordinates, no hand-authored route — see that module's docstring for the conformance
rationale and docs/parcours.md §7bis–§7quater.

Env (optional): IA712_INSPECT_GRID (N, default 2), IA712_INSPECT_OFFSET (m, default 1.3),
IA712_INSPECT_MIN_CELLS (default 80).
"""
import os
import sys

# Prefer the installed package; fall back to the in-tree source so the CLI also works
# straight from a checkout before `colcon build`.
try:
    from rescue_robot.navigation.inspection_planner import generate
except ImportError:
    _src = os.path.join(os.path.dirname(__file__), "..", "ros2_ws", "src",
                        "rescue_robot")
    sys.path.insert(0, os.path.abspath(_src))
    from rescue_robot.navigation.inspection_planner import generate


def main():
    results_dir = sys.argv[1] if len(sys.argv) > 1 else "results"
    out = sys.argv[2] if len(sys.argv) > 2 else os.path.join(results_dir, "inspection_waypoints.yaml")
    grid = int(os.environ.get("IA712_INSPECT_GRID", "2"))
    offset = float(os.environ.get("IA712_INSPECT_OFFSET", "1.3"))
    min_cells = int(os.environ.get("IA712_INSPECT_MIN_CELLS", "80"))

    wps = generate(results_dir, grid=grid, offset=offset, min_cells=min_cells)
    lines = [
        "# AUTONOMOUS room-inspection waypoints — GENERATED AT RUNTIME from the robot's",
        "# own SLAM map (final_map.pgm). No victim coordinates, no hand-authored route:",
        "# one pose per discovered outer room, facing its wall, swept by a short spin.",
        "# See rescue_robot/navigation/inspection_planner.py. Regenerated every run.",
        "waypoints:",
    ]
    for wp in wps:
        lines.append(f"  - {{x: {wp['x']}, y: {wp['y']}, yaw: {wp['yaw']}, dwell: true}}")
    with open(out, "w") as f:
        f.write("\n".join(lines) + "\n")
    print(f"[inspection] wrote {out} — {len(wps)} room-inspection pose(s) derived from the map:")
    for wp in wps:
        print(f"  ({wp['x']:.2f}, {wp['y']:.2f}) yaw={wp['yaw']:.2f}")


if __name__ == "__main__":
    main()
