#!/usr/bin/env python3
"""Generate AprilTag (tag36h11) Gazebo models using colored-box voxels.

PBR albedo_map textures are silently ignored by Ogre2+Mesa/D3D12 on WSL2 — the
panel renders as a blank white rectangle and apriltag_ros sees nothing.
Instead, each tag is built from 10×10 = 100 tiny black/white box visuals using
plain <ambient>/<diffuse> colors, which render correctly on all Ogre2 backends
(same mechanism as the rubble blocks in the scene).

Model naming follows dev/bl convention: apriltag_36h11_00 … 03.
The script also patches rescue_arena.sdf: red-cylinder victim placeholders are
replaced by <include> blocks that place each tag against an outer wall at camera
height (OAK-D ≈ 0.47 m → tag at 0.28 m, comfortably in the ±27° vertical FOV).

Usage:
    python3 scripts/generate_apriltag_models.py
    ./scripts/run.sh build
"""
from __future__ import annotations
import math
import re
import sys
from pathlib import Path

try:
    import cv2
    import numpy as np
except ImportError:
    sys.exit("pip install opencv-python numpy")

REPO_ROOT  = Path(__file__).resolve().parents[1]
MODELS_DIR = REPO_ROOT / "ros2_ws/src/rescue_world/models"
WORLD_SDF  = REPO_ROOT / "ros2_ws/src/rescue_world/worlds/rescue_arena.sdf"

TAG_FAMILY   = cv2.aruco.DICT_APRILTAG_36h11
TAG_SIZE_M   = 0.16
TAG_IDS      = [0, 1, 2, 3]
TAG_NAME_FMT = "apriltag_36h11_{:02d}"   # apriltag_36h11_00 … 03
GRID_N       = 10

# (tag_id, x, y, z, roll_deg, pitch_deg, yaw_deg)
# roll=+90 → face normal = -Y (south) : tag on north outer wall (y ≈ +5.924)
# roll=-90 → face normal = +Y (north) : tag on south outer wall (y ≈ -5.924)
VICTIM_POSES = [
    (0,  4.6,  5.924, 0.28,  90, 0, 0),   # NE room — north outer wall, facing south
    (1, -4.6, -5.924, 0.28, -90, 0, 0),   # SW room — south outer wall, facing north
    (2, -4.6,  5.924, 0.28,  90, 0, 0),   # NW room — north outer wall, facing south
    (3,  4.6, -5.924, 0.28, -90, 0, 0),   # SE room — south outer wall, facing north
]


def _tag_cells(tag_id: int) -> list[list[bool]]:
    """Return GRID_N×GRID_N grid: True=black, False=white."""
    aruco_dict = cv2.aruco.getPredefinedDictionary(TAG_FAMILY)
    img = np.zeros((200, 200), dtype=np.uint8)
    if hasattr(cv2.aruco, 'generateImageMarker'):
        cv2.aruco.generateImageMarker(aruco_dict, tag_id, 200, img, 1)
    else:
        cv2.aruco.drawMarker(aruco_dict, tag_id, 200, img, 1)
    small = cv2.resize(img, (GRID_N, GRID_N), interpolation=cv2.INTER_NEAREST)
    return [[bool(small[r, c] < 128) for c in range(GRID_N)] for r in range(GRID_N)]


def generate_model_sdf(tag_id: int) -> str:
    tag_name = TAG_NAME_FMT.format(tag_id)
    cells  = _tag_cells(tag_id)
    cell_w = TAG_SIZE_M / GRID_N
    half   = TAG_SIZE_M / 2.0
    gap    = 5e-5

    visuals = []
    for row, row_cells in enumerate(cells):
        for col, is_black in enumerate(row_cells):
            color = "0 0 0 1" if is_black else "1 1 1 1"
            x    = -half + (col + 0.5) * cell_w
            y    =  half - (row + 0.5) * cell_w
            side = cell_w - gap
            visuals.append(f"""
      <visual name="c{row:02d}{col:02d}">
        <pose>{x:.6f} {y:.6f} 0 0 0 0</pose>
        <geometry><box><size>{side:.6f} {side:.6f} 0.002</size></box></geometry>
        <material>
          <ambient>{color}</ambient>
          <diffuse>{color}</diffuse>
          <specular>0 0 0 1</specular>
        </material>
      </visual>""")

    cells_xml = ''.join(visuals)
    return f"""<?xml version="1.0"?>
<sdf version="1.8">
  <model name="{tag_name}">
    <static>true</static>
    <link name="link">{cells_xml}
      <collision name="collision">
        <geometry>
          <box><size>{TAG_SIZE_M} {TAG_SIZE_M} 0.003</size></box>
        </geometry>
      </collision>
    </link>
  </model>
</sdf>
"""


def pose_to_sdf(tag_id: int, name: str,
                x: float, y: float, z: float,
                roll_deg: float, pitch_deg: float, yaw_deg: float) -> str:
    tag_model = TAG_NAME_FMT.format(tag_id)
    r  = math.radians(roll_deg)
    p  = math.radians(pitch_deg)
    yw = math.radians(yaw_deg)
    return f"""
        <include>
            <uri>model://{tag_model}</uri>
            <name>{name}</name>
            <pose>{x:.4f} {y:.4f} {z:.4f} {r:.4f} {p:.4f} {yw:.4f}</pose>
        </include>"""


def main() -> None:
    MODELS_DIR.mkdir(parents=True, exist_ok=True)

    for tag_id in TAG_IDS:
        tag_name  = TAG_NAME_FMT.format(tag_id)
        model_dir = MODELS_DIR / tag_name
        model_dir.mkdir(exist_ok=True)
        (model_dir / "model.sdf").write_text(generate_model_sdf(tag_id))
        (model_dir / "model.config").write_text(f"""<?xml version="1.0"?>
<model>
  <name>{tag_name}</name>
  <version>1.0</version>
  <sdf version="1.8">model.sdf</sdf>
  <description>AprilTag tag36h11 ID {tag_id} — colored-box voxels (WSL2/Ogre2 safe)</description>
</model>
""")
        print(f"  model {tag_name}  ({GRID_N}x{GRID_N} = {GRID_N*GRID_N} cells)")

    includes = []
    for tag_id, x, y, z, roll, pitch, yaw in VICTIM_POSES:
        includes.append(pose_to_sdf(tag_id, f"victim_{tag_id}", x, y, z, roll, pitch, yaw))

    world_text = WORLD_SDF.read_text()
    world_text = re.sub(
        r"\s*<model name='victim_\d+'>\s*<static>1</static>.*?</model>",
        '', world_text, flags=re.DOTALL)
    world_text = re.sub(
        r'\s*<include>\s*<uri>model://apriltag_.*?</include>',
        '', world_text, flags=re.DOTALL)
    world_text = re.sub(
        r'\s*<!-- AprilTag victims.*?-->', '', world_text, flags=re.DOTALL)

    inject = "\n        <!-- AprilTag victims (auto-generated) -->" + "".join(includes) + "\n"
    world_text = world_text.replace("    </world>", inject + "    </world>")
    WORLD_SDF.write_text(world_text)
    print(f"  SDF   {WORLD_SDF.relative_to(REPO_ROOT)}")
    print("\nDone — relance avec : IA712_TB4_WORLD=rescue_arena ./scripts/run.sh demo-tb4")


if __name__ == "__main__":
    main()
