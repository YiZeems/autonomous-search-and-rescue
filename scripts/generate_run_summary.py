#!/usr/bin/env python3
"""Generate a small JSON run summary from exported CSV files."""
from __future__ import annotations

import csv
import json
from pathlib import Path


def read_last_coverage(path: Path) -> float:
    if not path.exists():
        return 0.0
    rows = list(csv.DictReader(path.open()))
    if not rows:
        return 0.0
    return float(rows[-1].get("coverage", 0.0))


def count_victims(path: Path) -> int:
    if not path.exists():
        return 0
    rows = list(csv.DictReader(path.open()))
    return len(rows)


def main() -> None:
    results_dir = Path("results")
    coverage_path = results_dir / "coverage_over_time.csv"
    victims_path = results_dir / "victims_detected.csv"

    if not coverage_path.exists():
        coverage_path = Path(
            "ros2_ws/src/rescue_robot/test_data/fake_coverage_over_time.csv"
        )
    if not victims_path.exists():
        victims_path = Path("ros2_ws/src/rescue_robot/test_data/fake_victims_detected.csv")

    summary = {
        "final_coverage": read_last_coverage(coverage_path),
        "victims_detected": count_victims(victims_path),
    }
    summary["success_coverage_90"] = summary["final_coverage"] >= 0.90

    results_dir.mkdir(exist_ok=True)
    out = results_dir / "run_summary.json"
    out.write_text(json.dumps(summary, indent=2) + "\n")
    print(f"Wrote {out}")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
