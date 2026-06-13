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


# ---------------------------------------------------------------------------
# blacklist_key + filter_blacklisted (CM8 inaccessible-frontier handling)
# ---------------------------------------------------------------------------

def test_blacklist_key_quantises_to_buckets() -> None:
    # points within the same 0.5 m bucket share a key; across buckets differ
    assert fs.blacklist_key(6.01, -3.20, 0.5) == fs.blacklist_key(6.04, -3.19, 0.5)
    assert fs.blacklist_key(6.01, -3.20, 0.5) != fs.blacklist_key(7.00, -3.20, 0.5)


def test_filter_blacklisted_empty_blacklist_is_noop() -> None:
    centroids = [(0.0, 0.0, 10), (5.0, 5.0, 8)]
    out = fs.filter_blacklisted(centroids, 0.05, 0.0, 0.0, set())
    assert out == centroids


def test_filter_blacklisted_drops_matching_world_key() -> None:
    # cell (10,10) at res 0.5 origin 0,0 -> world centre (5.25, 5.25) -> bucket (11,11)
    centroids = [(10.0, 10.0, 20), (0.0, 0.0, 15)]
    wx, wy = fs.cell_to_world(10.0, 10.0, 0.5, 0.0, 0.0)
    bl = {fs.blacklist_key(wx, wy, 0.5)}
    out = fs.filter_blacklisted(centroids, 0.5, 0.0, 0.0, bl, 0.5)
    assert (10.0, 10.0, 20) not in out
    assert (0.0, 0.0, 15) in out


def test_blacklisted_frontier_is_not_chosen() -> None:
    # the big near frontier is blacklisted -> the smaller far one wins instead
    centroids = [(1.0, 0.0, 50), (20.0, 0.0, 5)]
    wx, wy = fs.cell_to_world(1.0, 0.0, 1.0, 0.0, 0.0)
    bl = {fs.blacklist_key(wx, wy, 0.5)}
    reachable = fs.filter_blacklisted(centroids, 1.0, 0.0, 0.0, bl, 0.5)
    choice = fs.choose_frontier(reachable, (0.0, 0.0), min_size=3)
    assert choice == (20.0, 0.0, 5)


# ---------------------------------------------------------------------------
# L17 bonus: greedy vs information-gain selection
# ---------------------------------------------------------------------------

def _grid(width: int, height: int, fill: int = 0) -> list[int]:
    return [fill] * (width * height)


def test_count_unknown_in_radius() -> None:
    w, h = 5, 5
    data = _grid(w, h, 0)
    for x, y in [(2, 2), (2, 3), (3, 2), (1, 2), (2, 1)]:  # center + 4 orthogonal
        data[y * w + x] = fs.UNKNOWN
    # radius 1 => dx^2+dy^2<=1 => center + 4 neighbours, all unknown here
    assert fs.count_unknown_in_radius(data, w, h, 2, 2, 1) == 5
    assert fs.count_unknown_in_radius(data, w, h, 2, 2, 0) == 0  # radius 0 -> none


def test_choose_frontier_greedy_picks_nearest() -> None:
    cents = [(10, 0, 5), (2, 0, 5), (5, 5, 50)]  # nearest is (2,0) despite small size
    assert fs.choose_frontier_greedy(cents, (0, 0), min_size=3) == (2, 0, 5)


def test_choose_frontier_greedy_respects_min_size() -> None:
    cents = [(1, 0, 2), (4, 0, 5)]  # nearest too small -> the next one
    assert fs.choose_frontier_greedy(cents, (0, 0), min_size=3) == (4, 0, 5)


def test_infogain_balances_gain_and_cost() -> None:
    w, h = 20, 20
    data = _grid(w, h, 0)
    for y in range(12, 19):           # big unknown blob near (15,15)
        for x in range(12, 19):
            data[y * w + x] = fs.UNKNOWN
    cents = [(3, 3, 5), (12, 12, 5)]  # close+low-gain  vs  far+high-gain
    # small lambda: information dominates -> pick the far, high-gain frontier
    assert fs.choose_frontier_infogain(cents, (0, 0), data, w, h, 4, lam=0.1, min_size=3) == (12, 12, 5)
    # huge lambda: cost dominates -> behaves greedy, pick the close one
    assert fs.choose_frontier_infogain(cents, (0, 0), data, w, h, 4, lam=100.0, min_size=3) == (3, 3, 5)


def test_infogain_cost_fn_none_skips_unreachable() -> None:
    w, h = 10, 10
    data = _grid(w, h, 0)
    cents = [(5, 5, 5)]
    # cost_fn returning None == no Nav2 path -> frontier skipped -> nothing selected
    assert fs.choose_frontier_infogain(
        cents, (0, 0), data, w, h, 3, cost_fn=lambda x, y: None
    ) is None


def test_select_frontier_dispatch() -> None:
    cents = [(2, 0, 5), (8, 0, 5)]
    assert fs.select_frontier("greedy", cents, (0, 0), min_size=3) == (2, 0, 5)
    assert fs.select_frontier("size_dist", cents, (0, 0), min_size=3) is not None
    data = _grid(10, 10, fs.UNKNOWN)
    assert fs.select_frontier(
        "info_gain", cents, (0, 0), data=data, width=10, height=10,
        radius_cells=3, lam=1.0, min_size=3,
    ) is not None


def test_nearest_free_cell_snaps_to_free() -> None:
    w, h = 7, 7
    data = _grid(w, h, fs.UNKNOWN)        # all unknown...
    data[3 * w + 5] = 0                    # ...except one free cell at (5,3)
    # centroid sits in unknown (3,3) -> snap to the nearest free cell (5,3)
    assert fs.nearest_free_cell(data, w, h, 3, 3, search_radius=8) == (5, 3)
    # already free -> returns itself
    assert fs.nearest_free_cell(data, w, h, 5, 3) == (5, 3)
    # nothing free within radius -> None
    assert fs.nearest_free_cell(data, w, h, 0, 0, search_radius=1) is None


def test_infogain_min_gain_filters_low_gain() -> None:
    w, h = 20, 20
    data = _grid(w, h, 0)
    for y in range(12, 19):
        for x in range(12, 19):
            data[y * w + x] = fs.UNKNOWN     # high-gain blob near (15,15)
    cents = [(3, 3, 5), (12, 12, 5)]         # low-gain vs high-gain
    # min_gain high enough to drop the low-gain (3,3) -> only (12,12) eligible
    g = fs.choose_frontier_infogain(cents, (0, 0), data, w, h, 4, lam=0.1,
                                    min_size=3, min_gain=5)
    assert g == (12, 12, 5)
