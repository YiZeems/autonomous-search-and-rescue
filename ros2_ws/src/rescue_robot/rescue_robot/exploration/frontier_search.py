"""Pure frontier-detection logic for autonomous exploration.

Kept free of ROS imports so the algorithm is unit-testable on any machine
(no GPU / no sim needed). The node (frontier_explorer_node) wraps these
functions to turn an OccupancyGrid into the next NavigateToPose goal.

OccupancyGrid convention: -1 = unknown, 0 = free, 100 = occupied (0..100 cost).
A *frontier* cell is a free cell adjacent (4-connectivity) to an unknown cell —
i.e. the boundary between explored and unexplored space. The robot drives to
frontier clusters to expand the map until none remain.
"""
from __future__ import annotations

from collections import deque

UNKNOWN = -1
# A cell is "free enough" to stand on / be a frontier if its occupancy is low.
FREE_MAX = 25


def is_free(value: int) -> bool:
    return 0 <= value <= FREE_MAX


def find_frontier_cells(
    data, width: int, height: int
) -> list[tuple[int, int]]:
    """Return (x, y) of every free cell with >=1 unknown 4-neighbour."""
    if width <= 0 or height <= 0 or len(data) < width * height:
        return []
    frontiers: list[tuple[int, int]] = []
    for y in range(height):
        row = y * width
        for x in range(width):
            if not is_free(data[row + x]):
                continue
            for nx, ny in ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)):
                if 0 <= nx < width and 0 <= ny < height and data[ny * width + nx] == UNKNOWN:
                    frontiers.append((x, y))
                    break
    return frontiers


def cluster_frontiers(
    cells: list[tuple[int, int]]
) -> list[list[tuple[int, int]]]:
    """Group frontier cells into connected clusters (8-connectivity, BFS)."""
    remaining = set(cells)
    clusters: list[list[tuple[int, int]]] = []
    while remaining:
        start = remaining.pop()
        cluster = [start]
        queue = deque([start])
        while queue:
            cx, cy = queue.popleft()
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    if dx == 0 and dy == 0:
                        continue
                    neighbour = (cx + dx, cy + dy)
                    if neighbour in remaining:
                        remaining.discard(neighbour)
                        cluster.append(neighbour)
                        queue.append(neighbour)
        clusters.append(cluster)
    return clusters


def cluster_centroids(
    clusters: list[list[tuple[int, int]]], min_size: int = 1
) -> list[tuple[float, float, int]]:
    """Return (cx, cy, size) per cluster with at least ``min_size`` cells."""
    out: list[tuple[float, float, int]] = []
    for cluster in clusters:
        size = len(cluster)
        if size < min_size:
            continue
        sx = sum(c[0] for c in cluster)
        sy = sum(c[1] for c in cluster)
        out.append((sx / size, sy / size, size))
    return out


def cell_to_world(
    cx: float, cy: float, resolution: float, origin_x: float, origin_y: float
) -> tuple[float, float]:
    """Convert map-grid coordinates to world (map-frame) metres (cell centre)."""
    wx = origin_x + (cx + 0.5) * resolution
    wy = origin_y + (cy + 0.5) * resolution
    return wx, wy


def blacklist_key(
    wx: float, wy: float, quantum: float = 0.5
) -> tuple[int, int]:
    """Quantise a world (map-frame) point to a stable blacklist bucket.

    Frontiers are blacklisted in WORLD metres, not grid cells, because the grid
    origin/size shifts as SLAM grows the map — a grid-cell key would drift and
    a blacklisted unreachable frontier would silently come back. ``quantum`` is
    the bucket edge in metres (0.5 m groups near-identical re-selected goals).
    """
    return (round(wx / quantum), round(wy / quantum))


def filter_blacklisted(
    centroids: list[tuple[float, float, int]],
    resolution: float,
    origin_x: float,
    origin_y: float,
    blacklist: set[tuple[int, int]],
    quantum: float = 0.5,
) -> list[tuple[float, float, int]]:
    """Drop centroids whose world-quantised key is in ``blacklist``.

    Centroids are in grid coords; converted to world via ``cell_to_world`` then
    matched against the (world-quantised) blacklist.
    """
    if not blacklist:
        return list(centroids)
    out: list[tuple[float, float, int]] = []
    for cx, cy, size in centroids:
        wx, wy = cell_to_world(cx, cy, resolution, origin_x, origin_y)
        if blacklist_key(wx, wy, quantum) in blacklist:
            continue
        out.append((cx, cy, size))
    return out


def choose_frontier(
    centroids: list[tuple[float, float, int]],
    robot_cell: tuple[float, float],
    min_size: int = 3,
) -> tuple[float, float, int] | None:
    """Pick the best frontier centroid to drive to.

    Heuristic: among clusters of at least ``min_size`` cells, prefer the one
    with the best size/distance trade-off (large and close). Returns the chosen
    (cx, cy, size) in grid coords, or None when exploration is complete.
    """
    rx, ry = robot_cell
    best = None
    best_score = float("-inf")
    for cx, cy, size in centroids:
        if size < min_size:
            continue
        dist = ((cx - rx) ** 2 + (cy - ry) ** 2) ** 0.5
        # favour larger clusters, penalise distance; +1 avoids div-by-zero
        score = size / (dist + 1.0)
        if score > best_score:
            best_score = score
            best = (cx, cy, size)
    return best


# ── Two comparable exploration strategies ─────────────────────────
# The assignment bonus asks for a quantitative comparison of "greedy frontier"
# vs "information-gain" exploration (coverage over time). Both reuse the frontier
# detection/clustering above; only the GOAL-SELECTION rule differs:
#   greedy     : drive to the nearest reachable frontier (classic explore_lite).
#   info_gain  : maximise  gain(f) - lambda*cost(f)  (Stachniss et al., ICRA 2005)
#                gain = unknown cells revealed near f; cost = travel cost to f.
# Keep these PURE (no ROS) so they stay unit-testable; the node injects a Nav2
# path-length cost via ``cost_fn`` when available, else Euclidean is used.


def count_unknown_in_radius(
    data, width: int, height: int, cx: float, cy: float, radius_cells: int
) -> int:
    """Number of UNKNOWN cells within ``radius_cells`` of grid point (cx, cy).

    Proxy for the information a frontier would reveal once reached.
    """
    if radius_cells <= 0 or width <= 0 or height <= 0:
        return 0
    icx, icy = int(round(cx)), int(round(cy))
    r2 = radius_cells * radius_cells
    x0, x1 = max(0, icx - radius_cells), min(width - 1, icx + radius_cells)
    y0, y1 = max(0, icy - radius_cells), min(height - 1, icy + radius_cells)
    count = 0
    for y in range(y0, y1 + 1):
        row = y * width
        dy = y - icy
        for x in range(x0, x1 + 1):
            dx = x - icx
            if dx * dx + dy * dy <= r2 and data[row + x] == UNKNOWN:
                count += 1
    return count


def nearest_free_cell(
    data, width: int, height: int, cx: float, cy: float, search_radius: int = 8
) -> tuple[int, int] | None:
    """Nearest FREE grid cell to (cx, cy) within ``search_radius`` (or None).

    A frontier centroid often sits ON the explored/unknown boundary, so sending
    it raw makes Nav2 reject the goal (it lands in unknown/occupied space). We
    snap the goal to the closest free cell instead.
    """
    if width <= 0 or height <= 0:
        return None
    icx, icy = int(round(cx)), int(round(cy))
    if 0 <= icx < width and 0 <= icy < height and is_free(data[icy * width + icx]):
        return (icx, icy)
    for r in range(1, search_radius + 1):
        best = None
        best_d2 = None
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                if max(abs(dx), abs(dy)) != r:   # only the ring at radius r
                    continue
                x, y = icx + dx, icy + dy
                if 0 <= x < width and 0 <= y < height and is_free(data[y * width + x]):
                    d2 = dx * dx + dy * dy
                    if best_d2 is None or d2 < best_d2:
                        best_d2, best = d2, (x, y)
        if best is not None:
            return best                          # nearest ring with a free cell
    return None


def choose_frontier_greedy(
    centroids: list[tuple[float, float, int]],
    robot_cell: tuple[float, float],
    min_size: int = 3,
) -> tuple[float, float, int] | None:
    """Greedy baseline: the NEAREST frontier of at least ``min_size`` cells."""
    rx, ry = robot_cell
    best = None
    best_dist = float("inf")
    for cx, cy, size in centroids:
        if size < min_size:
            continue
        dist = ((cx - rx) ** 2 + (cy - ry) ** 2) ** 0.5
        if dist < best_dist:
            best_dist = dist
            best = (cx, cy, size)
    return best


def infogain_score(
    centroid: tuple[float, float, int],
    robot_cell: tuple[float, float],
    data, width: int, height: int,
    radius_cells: int, lam: float, cost_fn=None,
) -> tuple[int, float | None, float]:
    """Return (gain, cost, score) for one centroid.

    gain = unknown cells in ``radius_cells``; cost = ``cost_fn(cx, cy)`` if given
    (e.g. Nav2 path length; None means "unreachable"), else Euclidean cell
    distance; score = gain - lambda*cost.
    """
    cx, cy, _ = centroid
    gain = count_unknown_in_radius(data, width, height, cx, cy, radius_cells)
    if cost_fn is not None:
        cost = cost_fn(cx, cy)
        if cost is None:
            return gain, None, float("-inf")
    else:
        rx, ry = robot_cell
        cost = ((cx - rx) ** 2 + (cy - ry) ** 2) ** 0.5
    return gain, cost, gain - lam * cost


def choose_frontier_infogain(
    centroids: list[tuple[float, float, int]],
    robot_cell: tuple[float, float],
    data, width: int, height: int,
    radius_cells: int, lam: float = 1.0, min_size: int = 3, cost_fn=None,
    min_gain: int = 0,
) -> tuple[float, float, int] | None:
    """Information-gain selection: argmax over ``gain - lambda*cost``.

    Frontiers whose ``cost_fn`` returns None (no Nav2 path) are skipped, and so
    are frontiers revealing fewer than ``min_gain`` unknown cells (already-known
    areas not worth a trip).
    """
    best = None
    best_score = float("-inf")
    for c in centroids:
        if c[2] < min_size:
            continue
        gain, cost, score = infogain_score(
            c, robot_cell, data, width, height, radius_cells, lam, cost_fn
        )
        if cost is None or gain < min_gain:
            continue
        if score > best_score:
            best_score = score
            best = c
    return best


def select_frontier(
    strategy: str,
    centroids: list[tuple[float, float, int]],
    robot_cell: tuple[float, float],
    *, data=None, width: int = 0, height: int = 0,
    radius_cells: int = 0, lam: float = 1.0, min_size: int = 3, cost_fn=None,
    min_gain: int = 0,
) -> tuple[float, float, int] | None:
    """Dispatch goal selection on ``strategy`` (greedy | info_gain | size_dist)."""
    if strategy == "greedy":
        return choose_frontier_greedy(centroids, robot_cell, min_size)
    if strategy == "info_gain":
        return choose_frontier_infogain(
            centroids, robot_cell, data, width, height,
            radius_cells, lam, min_size, cost_fn, min_gain,
        )
    return choose_frontier(centroids, robot_cell, min_size)  # size/distance hybrid
