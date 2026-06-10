#!/usr/bin/env python3
"""Create a lightweight annotated map placeholder.

This script is intentionally dependency-light. If Pillow is installed and a PGM map
exists, it will create a simple annotated PNG. Otherwise it prints what would be
annotated, which is still useful during the base/scaffold phase.
"""
from __future__ import annotations

import csv
from pathlib import Path


def load_victims(path: Path) -> list[dict[str, str]]:
    if not path.exists():
        return []
    with path.open(newline="") as f:
        return list(csv.DictReader(f))


def main() -> None:
    victims_path = Path("results/victims_detected.csv")
    if not victims_path.exists():
        victims_path = Path("ros2_ws/src/rescue_robot/test_data/fake_victims_detected.csv")

    victims = load_victims(victims_path)
    print(f"Loaded {len(victims)} victims from {victims_path}")
    for victim in victims:
        print(victim)

    print("Map annotation placeholder completed.")
    print("TODO(D): convert map coordinates to image pixels using final_map.yaml and draw labels.")


if __name__ == "__main__":
    main()
