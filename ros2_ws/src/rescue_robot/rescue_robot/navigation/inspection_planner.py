"""Plan AUTONOMOUS room-inspection poses from the robot's own SLAM map.

Conformance (IA712 Projet B — "explore an UNKNOWN environment, locate victims,
WITHOUT human intervention"): the ONLY input is the map the robot built itself —
NO victim coordinates, NO hand-authored route. The free space is segmented into
regions; each OUTER room gets ONE inspection pose ~`offset` m off its nearest wall,
facing that wall, snapped to a navigable free cell, the set ordered as a perimeter
loop. Driven with a short camera sweep, this is systematic perimeter search.

Two entry points share one core (`_compute_poses`):
  - `generate(results_dir)`   reads final_map.{yaml,pgm}  (the standalone CLI / shell)
  - `poses_from_grid(...)`    reads a live ROS OccupancyGrid (the inspection_node, BT)
"""
import math
import os
import re


def _load_yaml(path):
    res, ox, oy, image = 0.05, 0.0, 0.0, "final_map.pgm"
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
    idx, tokens = 0, []
    while len(tokens) < 4:
        while idx < len(data) and data[idx:idx + 1].isspace():
            idx += 1
        if data[idx:idx + 1] == b"#":
            while idx < len(data) and data[idx:idx + 1] not in (b"\n", b"\r"):
                idx += 1
            continue
        start = idx
        while idx < len(data) and not data[idx:idx + 1].isspace():
            idx += 1
        tokens.append(data[start:idx])
    magic, w, h, _maxval = tokens[0], int(tokens[1]), int(tokens[2]), int(tokens[3])
    assert magic == b"P5", f"only binary PGM (P5) supported, got {magic}"
    idx += 1
    pix = data[idx:idx + w * h]
    rows = [list(pix[r * w:(r + 1) * w]) for r in range(h)]
    return w, h, rows


def _classify_points(w, h, rows, res, ox, oy):
    """World (x, y) of free cells (>=250) and occupied/wall cells (<=50) from a PGM."""
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


def poses_from_grid(data, width, height, resolution, origin_x, origin_y,
                    grid=2, offset=1.3, min_cells=80):
    """Inspection poses from a live ROS OccupancyGrid (data row-major; 0=free, 100=occ,
    -1=unknown; origin at the bottom-left, row 0 = min y)."""
    free, occ = [], []
    for r in range(height):
        wy = origin_y + (r + 0.5) * resolution
        base = r * width
        for c in range(width):
            v = data[base + c]
            wx = origin_x + (c + 0.5) * resolution
            if 0 <= v < 25:
                free.append((wx, wy))
            elif v >= 65:
                occ.append((wx, wy))
    return _compute_poses(free, occ, grid, offset, min_cells)


def _compute_poses(pts, occ, grid=2, offset=1.3, min_cells=80):
    """Core: one inspection pose per outer room — face the nearest wall, snap to a
    navigable free cell, perimeter-loop order. See module docstring."""
    if not pts:
        return []
    cx = sum(p[0] for p in pts) / len(pts)
    cy = sum(p[1] for p in pts) / len(pts)

    # Robustness vs SLAM drift: a warped map can sprout spurious free cells far outside
    # the arena. Clip the working extent to a high percentile of |coord| and drop the rest.
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
            continue
        cells_by_d = sorted(cells, key=lambda p: (p[0] - cx) ** 2 + (p[1] - cy) ** 2)
        outer = cells_by_d[int(0.95 * (len(cells_by_d) - 1))]
        dx, dy = outer[0] - cx, outer[1] - cy
        d = math.hypot(dx, dy) or 1.0
        ux, uy = dx / d, dy / d
        px, py = outer[0] - ux * offset, outer[1] - uy * offset
        # Snap onto the nearest actually-free mapped cell → guaranteed navigable.
        fx, fy = min(pts, key=lambda p: (p[0] - px) ** 2 + (p[1] - py) ** 2)
        px, py = fx, fy
        # Aim straight at the nearest wall (the tag is ON it) → a short sweep suffices.
        if occ:
            wx, wy = min(occ, key=lambda p: (p[0] - px) ** 2 + (p[1] - py) ** 2)
            yaw = math.atan2(wy - py, wx - px)
        else:
            yaw = math.atan2(uy, ux)
        waypoints.append({"x": round(px, 2), "y": round(py, 2), "yaw": round(yaw, 4),
                          "dwell": True, "_ang": math.atan2(py - cy, px - cx)})
    # Perimeter-loop order (by angle) → minimal driving → minimal drift.
    waypoints.sort(key=lambda wp: wp["_ang"])
    for wp in waypoints:
        wp.pop("_ang", None)
    return waypoints


def generate(results_dir, grid=2, offset=1.3, min_cells=80):
    res, ox, oy, image = _load_yaml(os.path.join(results_dir, "final_map.yaml"))
    w, h, rows = _read_pgm(os.path.join(results_dir, image))
    pts, occ = _classify_points(w, h, rows, res, ox, oy)
    return _compute_poses(pts, occ, grid, offset, min_cells)
