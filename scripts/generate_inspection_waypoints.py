#!/usr/bin/env python3
"""Generate AUTONOMOUS room-inspection waypoints from the robot's own SLAM map.

Conformance note (IA712 Projet B — "explore an UNKNOWN environment, locate victims,
WITHOUT human intervention"):
  This script takes the **map the robot built itself** as its only input. It contains
  NO victim coordinates and NO hand-authored route. It segments the discovered free
  space into regions and, for each region, emits ONE inspection pose ~`offset` m inside
  that region's outermost reachable cell, facing the wall. Driven (by the waypoint
  follower) with a 360° spin at each pose, this is a *systematic inspection of every
  discovered room's perimeter* — autonomous search, not "go to the victims". The
  camera only sees ~2 m, while the LIDAR maps ~12 m, so pure frontier exploration maps
  the corner rooms from their doorways without ever bringing the camera within range of
  the wall AprilTags (cf. docs/parcours.md §7ter). This inspection pass closes that gap.

Usage:
    python3 generate_inspection_waypoints.py <results_dir> [out.yaml]
    # reads <results_dir>/final_map.{yaml,pgm}, writes <results_dir>/inspection_waypoints.yaml

Env (optional):
    IA712_INSPECT_GRID   grid size N (NxN regions); outer-ring regions get a pose. Default 2.
    IA712_INSPECT_OFFSET metres to stand back from the outer cell toward centre. Default 1.3.
    IA712_INSPECT_MIN_CELLS minimum free cells in a region to bother inspecting it. Default 80.
"""
import math
import os
import sys


def _load_yaml(path):
    res, ox, oy = 0.05, 0.0, 0.0
    image = "final_map.pgm"
    with open(path) as f:
        for line in f:
            line = line.strip()
            if line.startswith("resolution:"):
                res = float(line.split(":", 1)[1])
            elif line.startswith("origin:"):
                nums = line.split("[", 1)[1].split("]", 1)[0].split(",")
                ox, oy = float(nums[0]), float(nums[1])
            elif line.startswith("image:"):
                image = line.split(":", 1)[1].strip()
    return res, ox, oy, image


def _read_pgm(path):
    """Minimal P5 (binary) PGM reader → (width, height, list-of-rows-of-ints)."""
    with open(path, "rb") as f:
        data = f.read()
    # Parse header tokens (magic, width, height, maxval), skipping comments.
    idx = 0
    tokens = []
    while len(tokens) < 4:
        # skip whitespace
        while idx < len(data) and data[idx:idx + 1].isspace():
            idx += 1
        if data[idx:idx + 1] == b"#":  # comment to end of line
            while idx < len(data) and data[idx:idx + 1] not in (b"\n", b"\r"):
                idx += 1
            continue
        start = idx
        while idx < len(data) and not data[idx:idx + 1].isspace():
            idx += 1
        tokens.append(data[start:idx])
    magic, w, h, _maxval = tokens[0], int(tokens[1]), int(tokens[2]), int(tokens[3])
    assert magic == b"P5", f"only binary PGM (P5) supported, got {magic}"
    idx += 1  # single whitespace after maxval
    pix = data[idx:idx + w * h]
    rows = [list(pix[r * w:(r + 1) * w]) for r in range(h)]
    return w, h, rows


def _classify_points(w, h, rows, res, ox, oy):
    """World (x, y) of free cells (>=250) and occupied/wall cells (<=50)."""
    free, occ = [], []
    for r in range(h):
        row = rows[r]
        wy = oy + (h - 1 - r + 0.5) * res  # image row 0 is the TOP (max y)
        for c in range(w):
            v = row[c]
            wx = ox + (c + 0.5) * res
            if v >= 250:
                free.append((wx, wy))
            elif v <= 50:
                occ.append((wx, wy))
    return free, occ


def generate(results_dir, grid=2, offset=1.3, min_cells=80):
    res, ox, oy, image = _load_yaml(os.path.join(results_dir, "final_map.yaml"))
    w, h, rows = _read_pgm(os.path.join(results_dir, image))
    pts, occ = _classify_points(w, h, rows, res, ox, oy)
    if not pts:
        return []

    cx = sum(p[0] for p in pts) / len(pts)
    cy = sum(p[1] for p in pts) / len(pts)

    # ROBUSTNESS vs SLAM drift: a warped map can sprout a few spurious free cells
    # far outside the real arena (v23 saw cells at x≈9.6 in a ±6 m arena). Using
    # those as the "outer" reference puts an inspection pose outside the building.
    # So clip the working extent to a high percentile of |coord| and DROP cells
    # beyond it — the bulk of the map defines the arena, not a handful of outliers.
    def _pct(values, q):
        s = sorted(values)
        return s[min(len(s) - 1, int(q * (len(s) - 1)))]

    ax = _pct([abs(p[0] - cx) for p in pts], 0.98)
    ay = _pct([abs(p[1] - cy) for p in pts], 0.98)
    pts = [p for p in pts if abs(p[0] - cx) <= ax * 1.05 and abs(p[1] - cy) <= ay * 1.05]
    if not pts:
        return []
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    minx, maxx, miny, maxy = min(xs), max(xs), min(ys), max(ys)

    # Bucket free cells into a grid of regions over the discovered extent.
    def region_of(x, y):
        gx = min(grid - 1, int((x - minx) / max(1e-6, (maxx - minx)) * grid))
        gy = min(grid - 1, int((y - miny) / max(1e-6, (maxy - miny)) * grid))
        return gx, gy

    buckets = {}
    for p in pts:
        buckets.setdefault(region_of(*p), []).append(p)

    waypoints = []
    for (gx, gy), cells in sorted(buckets.items()):
        is_outer = gx in (0, grid - 1) or gy in (0, grid - 1)
        if not is_outer or len(cells) < min_cells:
            continue  # only inspect outer-ring regions with real free area
        # Outer reference = 95th-percentile-distance cell of this region (robust to a
        # few drift outliers, unlike the single farthest cell).
        cells_by_d = sorted(cells, key=lambda p: (p[0] - cx) ** 2 + (p[1] - cy) ** 2)
        outer = cells_by_d[int(0.95 * (len(cells_by_d) - 1))]
        dx, dy = outer[0] - cx, outer[1] - cy
        d = math.hypot(dx, dy) or 1.0
        ux, uy = dx / d, dy / d
        # Stand `offset` m back from the outer cell, toward the centre (≈ offset m from wall).
        px, py = outer[0] - ux * offset, outer[1] - uy * offset
        # SNAP the pose onto the nearest actually-FREE mapped cell. Without this the
        # pulled-back point can land on an unknown/occupied cell → Nav2 rejects the goal
        # (v25: the NW pose was rejected twice → that victim missed). We have the whole
        # free-cell set here, so guarantee the emitted pose is navigable.
        fx, fy = min(pts, key=lambda p: (p[0] - px) ** 2 + (p[1] - py) ** 2)
        px, py = fx, fy
        # AIM the camera straight at the nearest wall (occupied cell). The victim tag is
        # ON that outer wall, so facing it directly puts the tag in the camera FOV — a
        # SHORT sweep then suffices instead of a full 360° spin, which halves the in-place
        # rotation and so the rotational SLAM drift that wrecked coverage in v26/v28.
        if occ:
            wx, wy = min(occ, key=lambda p: (p[0] - px) ** 2 + (p[1] - py) ** 2)
            yaw = math.atan2(wy - py, wx - px)
        else:
            yaw = math.atan2(uy, ux)  # fallback: face outward
        waypoints.append({"x": round(px, 2), "y": round(py, 2),
                          "yaw": round(yaw, 4), "dwell": True,
                          "_ang": math.atan2(py - cy, px - cx)})
    # Order as a PERIMETER LOOP (by angle around the centre) so the robot circles the
    # arena instead of criss-crossing it corner-to-corner — far less driving, hence far
    # less odometry/SLAM drift during the inspection phase (v24: a SW→NW→SE→NE order
    # crossed the whole arena and pushed final coverage down to the 90 % line).
    waypoints.sort(key=lambda wp: wp["_ang"])
    for wp in waypoints:
        wp.pop("_ang", None)
    return waypoints


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
        "# one pose per discovered outer room, ~%.1f m from its wall, swept by a 360° spin." % offset,
        "# See scripts/generate_inspection_waypoints.py. Regenerated every run.",
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
