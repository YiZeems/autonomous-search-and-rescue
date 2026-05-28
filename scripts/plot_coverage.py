#!/usr/bin/env python3
"""Generate a coverage curve if matplotlib is available, otherwise print data.

Input priority:
1. results/coverage_over_time.csv
2. ros2_ws/src/ia712_search_rescue/test_data/fake_coverage_over_time.csv
"""
from __future__ import annotations

from pathlib import Path
import csv


def read_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline='') as f:
        return list(csv.DictReader(f))


def main() -> None:
    path = Path('results/coverage_over_time.csv')
    if not path.exists():
        path = Path('ros2_ws/src/ia712_search_rescue/test_data/fake_coverage_over_time.csv')

    rows = read_rows(path)
    print(f'Loaded {len(rows)} coverage rows from {path}')
    if not rows:
        return

    times = [float(row['time']) for row in rows]
    coverage = [float(row['coverage']) for row in rows]
    print(f'Final coverage: {coverage[-1]:.3f}')

    try:
        import matplotlib.pyplot as plt  # type: ignore
    except Exception:
        print('matplotlib not available; skipping PNG generation.')
        for row in rows[-5:]:
            print(row)
        return

    out_dir = Path('results')
    out_dir.mkdir(exist_ok=True)
    out = out_dir / 'coverage_curve.png'
    plt.figure()
    plt.plot(times, coverage, marker='o')
    plt.xlabel('time (s)')
    plt.ylabel('coverage')
    plt.ylim(0, 1)
    plt.grid(True)
    plt.savefig(out, bbox_inches='tight')
    print(f'Wrote {out}')


if __name__ == '__main__':
    main()
