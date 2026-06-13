#!/usr/bin/env python3
"""Generate AprilTag (tag36h11) Gazebo models using colored-box voxels.

PBR albedo_map textures are silently ignored by Ogre2+Mesa/D3D12 on WSL2 — the
panel renders as a blank white rectangle and apriltag_ros sees nothing.
Instead, each tag is built from 10×10 = 100 tiny black/white box visuals using
plain <ambient>/<diffuse> colors, which render correctly on all Ogre2 backends
(same mechanism as the rubble blocks in the scene).

Model naming follows the apriltag_36h11_00 … 03 convention.
This script ONLY builds the tag model assets (models/apriltag_36h11_0X/). Victim
PLACEMENT in the world is owned by scripts/generate_rescue_arena.py, whose
rescue_arena.sdf <include>s these models against an outer wall at camera height
(OAK-D ≈ 0.47 m → tag at 0.28 m, comfortably in the ±27° vertical FOV).

Usage:
    python3 scripts/generate_apriltag_models.py
    ./scripts/run.sh build
"""
from __future__ import annotations
import sys
from pathlib import Path

try:
    import cv2
    import numpy as np
except ImportError:
    sys.exit("pip install opencv-python numpy")

REPO_ROOT  = Path(__file__).resolve().parents[1]
MODELS_DIR = REPO_ROOT / "ros2_ws/src/rescue_world/models"

TAG_FAMILY   = cv2.aruco.DICT_APRILTAG_36h11
TAG_SIZE_M   = 0.16
TAG_IDS      = [0, 1, 2, 3]
TAG_NAME_FMT = "apriltag_36h11_{:02d}"   # apriltag_36h11_00 … 03
GRID_N       = 10


def _tag_cells(tag_id: int) -> list[list[bool]]:
    """Return GRID_N×GRID_N grid: True=black, False=white.

    Layout: a 1-cell WHITE quiet-zone ring (the AprilTag detector needs it to
    locate the tag's outer black border) around the native (markerSize+2)=8×8
    tag36h11 image. GRID_N is therefore 10.

    IMPORTANT: do NOT cv2.resize the 200px marker straight to 10×10 — that maps
    10 sample points onto the 8 real cells, duplicating two columns/rows and
    distorting the code so the detector rejects it (verified: such a tag renders
    fine but is undetectable). Downsample to the marker's true 8×8, then pad with
    the white ring.
    """
    aruco_dict = cv2.aruco.getPredefinedDictionary(TAG_FAMILY)
    img = np.zeros((200, 200), dtype=np.uint8)
    if hasattr(cv2.aruco, 'generateImageMarker'):
        cv2.aruco.generateImageMarker(aruco_dict, tag_id, 200, img, 1)
    else:
        cv2.aruco.drawMarker(aruco_dict, tag_id, 200, img, 1)
    marker_n = aruco_dict.markerSize + 2          # 6 data bits + black border = 8
    marker = cv2.resize(img, (marker_n, marker_n), interpolation=cv2.INTER_NEAREST)
    grid = np.full((GRID_N, GRID_N), 255, dtype=np.uint8)   # white quiet-zone ring
    off = (GRID_N - marker_n) // 2                # = 1 for GRID_N=10
    grid[off:off + marker_n, off:off + marker_n] = marker
    return [[bool(grid[r, c] < 128) for c in range(GRID_N)] for r in range(GRID_N)]


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

    print("\nDone — tag model assets written. Victim PLACEMENT is owned by "
          "scripts/generate_rescue_arena.py, whose rescue_arena.sdf already "
          "<include>s these models. Rebuild: ./scripts/run.sh build")


if __name__ == "__main__":
    main()
