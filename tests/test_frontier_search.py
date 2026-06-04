"""Unit tests for the pure frontier-detection algorithm (no ROS needed).

Real bugs these catch:
  - frontier detection misses the explored/unknown boundary -> robot never explores
  - clustering merges/splits wrong -> goals land in walls or oscillate
  - grid->world conversion off-by-half -> goals shifted by a cell, robot misses
  - choose_frontier ignores size/distance -> robot picks useless 1-cell frontiers
"""
from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "ros2_ws" / "src" / "rescue_robot"))

from rescue_robot.exploration import frontier_search as fs  # noqa: E402


# ---------------------------------------------------------------------------
# find_frontier_cells
# ---------------------------------------------------------------------------

def test_no_unknown_means_no_frontier() -> None:
    # all free, no unknown -> nothing to explore
    data = [0] * 25
    assert fs.find_frontier_cells(data, 5, 5) == []


def test_all_unknown_means_no_frontier() -> None:
    # no free cell to stand on
    data = [-1] * 25
    assert fs.find_frontier_cells(data, 5, 5) == []


def test_free_next_to_unknown_is_frontier() -> None:
    # left half free, right half unknown -> boundary column is frontier
    w, h = 4, 2
    data = [0, 0, -1, -1,
            0, 0, -1, -1]
    cells = set(fs.find_frontier_cells(data, w, h))
    assert (1, 0) in cells and (1, 1) in cells   # free cells touching unknown
    assert (0, 0) not in cells                    # free but not touching unknown


def test_occupied_cell_is_not_frontier() -> None:
    # occupied (100) next to unknown must NOT be a frontier (can't stand there)
    data = [100, -1,
            0, -1]
    cells = set(fs.find_frontier_cells(data, 2, 2))
    assert (0, 0) not in cells   # occupied
    assert (0, 1) in cells       # free, touches unknown above-right? check 4-neighbour


def test_malformed_grid_returns_empty() -> None:
    assert fs.find_frontier_cells([0, 0, 0], 5, 5) == []
    assert fs.find_frontier_cells([], 0, 0) == []


# ---------------------------------------------------------------------------
# clustering + centroids
# ---------------------------------------------------------------------------

def test_adjacent_cells_form_one_cluster() -> None:
    cells = [(1, 1), (1, 2), (2, 1)]
    clusters = fs.cluster_frontiers(cells)
    assert len(clusters) == 1
    assert len(clusters[0]) == 3


def test_separated_cells_form_multiple_clusters() -> None:
    cells = [(0, 0), (1, 0), (8, 8)]   # two near, one far
    clusters = fs.cluster_frontiers(cells)
    assert len(clusters) == 2


def test_centroid_and_size() -> None:
    clusters = [[(0, 0), (2, 0), (1, 3)]]
    out = fs.cluster_centroids(clusters)
    assert len(out) == 1
    cx, cy, size = out[0]
    assert size == 3
    assert abs(cx - 1.0) < 1e-9 and abs(cy - 1.0) < 1e-9


def test_centroid_min_size_filter() -> None:
    clusters = [[(0, 0)], [(5, 5), (5, 6), (6, 5)]]
    out = fs.cluster_centroids(clusters, min_size=3)
    assert len(out) == 1 and out[0][2] == 3


# ---------------------------------------------------------------------------
# grid -> world
# ---------------------------------------------------------------------------

def test_cell_to_world_centre_offset() -> None:
    # origin (-1,-2), resolution 0.5, cell (0,0) -> centre at (-0.75,-1.75)
    wx, wy = fs.cell_to_world(0, 0, 0.5, -1.0, -2.0)
    assert abs(wx - (-0.75)) < 1e-9 and abs(wy - (-1.75)) < 1e-9


def test_cell_to_world_scales_with_resolution() -> None:
    wx, wy = fs.cell_to_world(10, 4, 0.05, 0.0, 0.0)
    assert abs(wx - (10.5 * 0.05)) < 1e-9 and abs(wy - (4.5 * 0.05)) < 1e-9


# ---------------------------------------------------------------------------
# choose_frontier
# ---------------------------------------------------------------------------

def test_choose_none_when_empty() -> None:
    assert fs.choose_frontier([], (0, 0), min_size=3) is None


def test_choose_respects_min_size() -> None:
    # only small clusters -> nothing worth visiting
    assert fs.choose_frontier([(1.0, 1.0, 2)], (0.0, 0.0), min_size=3) is None


def test_choose_prefers_large_close_cluster() -> None:
    # big cluster nearby should beat a small far one
    near_big = (1.0, 0.0, 50)
    far_small = (20.0, 0.0, 5)
    choice = fs.choose_frontier([far_small, near_big], (0.0, 0.0), min_size=3)
    assert choice == near_big


def test_choose_picks_closer_when_same_size() -> None:
    a = (2.0, 0.0, 10)
    b = (9.0, 0.0, 10)
    choice = fs.choose_frontier([b, a], (0.0, 0.0), min_size=3)
    assert choice == a
