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
