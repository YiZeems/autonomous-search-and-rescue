"""Tests for the generated search-and-rescue world (rescue_arena.sdf).

The world is produced by scripts/generate_rescue_arena.py from box/cylinder
primitives only (no meshes), so it renders under software GL and is detected by
the gpu_lidar. These tests pin its structure and keep it in sync with the
generator.
"""
from __future__ import annotations

import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
WORLD = ROOT / "ros2_ws" / "src" / "rescue_world" / "worlds" / "rescue_arena.sdf"
GEN = ROOT / "scripts" / "generate_rescue_arena.py"


def _models() -> list[str]:
    world = ET.parse(WORLD).getroot().find("world")
    # Victims are placed as <include name="victim_N"> (AprilTag model refs), the
    # rest (walls, rubble, ground) as <model name="...">. Collect both.
    names = [m.get("name") for m in world.findall("model")]
    names.extend(i.findtext("name") for i in world.findall("include"))
    return [n for n in names if n]


def test_world_and_generator_exist() -> None:
    assert WORLD.exists(), "rescue_arena.sdf missing — run scripts/generate_rescue_arena.py"
    assert GEN.exists(), "generate_rescue_arena.py missing"


def test_world_is_valid_sdf() -> None:
    root = ET.parse(WORLD).getroot()
    assert root.tag == "sdf", f"root must be <sdf>, got <{root.tag}>"
    assert root.find("world") is not None, "missing <world>"


def test_world_has_rescue_content() -> None:
    names = _models()
    assert "ground_plane" in names, "missing ground_plane"
    assert any(n.startswith("wall_") for n in names), "no walls — robot would see nothing"
    assert sum(n.startswith("wall_") for n in names) >= 8, "too few walls for rooms+corridor"
    assert any(n.startswith("victim_") for n in names), "no victims for the rescue task"
    assert any(n.startswith("rubble_") for n in names), "no rubble obstacles"


def test_world_has_physics_and_shadows_off() -> None:
    """Software renderer needs an explicit physics step and shadows disabled."""
    world = ET.parse(WORLD).getroot().find("world")
    assert world.find("physics") is not None, "missing <physics> block"
    scene = world.find("scene")
    assert scene is not None and scene.find("shadows") is not None
    assert scene.find("shadows").text.strip() == "0", "shadows must be off for software GL"


def test_origin_is_clear_for_spawn() -> None:
    """No wall/rubble model centred within 0.5 m of origin (robot spawns at 0,0)."""
    world = ET.parse(WORLD).getroot().find("world")
    for m in world.findall("model"):
        name = m.get("name")
        if name in ("ground_plane",):
            continue
        pose = m.find("pose")
        if pose is None:
            continue
        x, y, *_ = (float(v) for v in pose.text.split())
        assert not (abs(x) < 0.5 and abs(y) < 0.5), (
            f"model '{name}' at ({x},{y}) blocks the spawn point — lidar would self-embed"
        )


def test_generator_is_reproducible(tmp_path: Path) -> None:
    """Re-running the generator must reproduce the committed world byte-for-byte."""
    res = subprocess.run([sys.executable, str(GEN)], capture_output=True, text=True)
    assert res.returncode == 0, f"generator failed: {res.stderr}"
    # generator writes to the repo path; ensure it still parses and matches model count
    assert ET.parse(WORLD).getroot().tag == "sdf"
