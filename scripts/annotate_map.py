#!/usr/bin/env python3
"""Annotate the final SLAM map with detected victim positions.

Deliverable for Projet B / L17-L18: a *final map marked with the victims*.

Reads, from a run directory (default ``results/``):
  - ``final_map.yaml`` + ``final_map.pgm``  (saved by ``nav2_map_server map_saver_cli``)
  - ``victims.json``                        (victim_registry_node: real AprilTag IDs + map x,y)
  - ``run_summary.json``                    (optional: final coverage for the legend)

and writes ``final_map_annotated.png`` with a circle + ``victim_<id>`` label at each
victim, using the standard occupancy-grid pixel mapping::

    px = (x - origin_x) / resolution
    py = image_height - (y - origin_y) / resolution      # PGM y axis points down

Usage:
    python3 scripts/annotate_map.py [RUN_DIR]

Dependency-light: needs Pillow (``PIL``) to draw. Without a map it falls back to
printing the victims it *would* annotate (still useful in the scaffold phase).
"""
from __future__ import annotations

import csv
import json
import sys
from pathlib import Path


def _load_map_yaml(path: Path) -> dict:
    """Parse the few fields we need from a nav2 map yaml (no PyYAML dependency)."""
    info: dict = {}
    for line in path.read_text().splitlines():
        line = line.split("#", 1)[0].strip()
        if not line or ":" not in line:
            continue
        key, val = line.split(":", 1)
        key, val = key.strip(), val.strip()
        if key == "image":
            info["image"] = val.strip("'\"")
        elif key == "resolution":
            info["resolution"] = float(val)
        elif key == "origin":
            nums = val.strip("[]").split(",")
            info["origin"] = [float(n) for n in nums[:3]]
    return info


def _load_victims(run_dir: Path) -> list[dict]:
    """Prefer victims.json (real AprilTag IDs); fall back to victims_detected.csv."""
    vj = run_dir / "victims.json"
    if vj.exists():
        data = json.loads(vj.read_text())
        # victim_registry_node writes {"frame": "map", "victims": [ {...}, ... ]};
        # also tolerate a bare list or an id-keyed dict {"0": {...}, ...}.
        if isinstance(data, dict):
            items = data["victims"] if "victims" in data else list(data.values())
        else:
            items = data
        out = []
        for v in items:
            if not isinstance(v, dict):
                continue
            x = v.get("x", (v.get("position") or {}).get("x"))
            y = v.get("y", (v.get("position") or {}).get("y"))
            if x is None or y is None:
                continue
            out.append({"id": str(v.get("id", v.get("victim_id", "?"))),
                        "x": float(x), "y": float(y)})
        return out
    csvp = run_dir / "victims_detected.csv"
    if csvp.exists():
        out = []
        with csvp.open(newline="") as f:
            for i, row in enumerate(csv.DictReader(f), 1):
                try:
                    out.append({"id": str(row.get("id", i)),
                                "x": float(row["x"]), "y": float(row["y"])})
                except (KeyError, ValueError):
                    continue
        return out
    return []


def annotate(run_dir: Path) -> int:
    yaml_path = run_dir / "final_map.yaml"
    victims = _load_victims(run_dir)

    if not yaml_path.exists():
        print(f"[annotate_map] no {yaml_path} — nothing to draw.")
        print(f"[annotate_map] {len(victims)} victim(s) would be annotated:")
        for v in victims:
            print(f"  victim_{v['id']}: ({v['x']:.2f}, {v['y']:.2f}) map")
        return 1

    try:
        from PIL import Image, ImageDraw
    except ImportError:
        print("[annotate_map] Pillow (PIL) not installed — cannot draw. "
              "Install python3-pil or `pip install pillow`.")
        return 2

    info = _load_map_yaml(yaml_path)
    res = info["resolution"]
    ox, oy = info["origin"][0], info["origin"][1]
    img = Image.open(run_dir / info["image"]).convert("RGB")
    W, H = img.size
    draw = ImageDraw.Draw(img)

    coverage = None
    rs = run_dir / "run_summary.json"
    if rs.exists():
        try:
            coverage = json.loads(rs.read_text()).get("final_coverage")
        except (json.JSONDecodeError, OSError):
            pass

    r = max(4, int(round(0.15 / res)))  # ~15 cm marker
    for v in victims:
        px = (v["x"] - ox) / res
        py = H - (v["y"] - oy) / res
        draw.ellipse([px - r, py - r, px + r, py + r], outline=(220, 30, 30), width=2)
        draw.line([px - r, py, px + r, py], fill=(220, 30, 30), width=1)
        draw.line([px, py - r, px, py + r], fill=(220, 30, 30), width=1)
        draw.text((px + r + 2, py - r), f"victim_{v['id']}", fill=(200, 0, 0))

    legend = [f"{len(victims)} victim(s)"]
    if coverage is not None:
        legend.append(f"coverage {coverage * 100:.0f}%")
    draw.text((4, 4), " | ".join(legend), fill=(0, 0, 160))

    out = run_dir / "final_map_annotated.png"
    img.save(out)
    print(f"[annotate_map] wrote {out}  ({len(victims)} victims, {W}x{H}px)")
    for v in victims:
        print(f"  victim_{v['id']}: map=({v['x']:.2f},{v['y']:.2f})")
    return 0


def main() -> None:
    run_dir = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("results")
    sys.exit(annotate(run_dir))


if __name__ == "__main__":
    main()
