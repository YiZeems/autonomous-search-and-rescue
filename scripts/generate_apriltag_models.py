#!/usr/bin/env python3
"""Generate Ignition Gazebo SDF models for AprilTag "victims" (tag36h11, IDs 0-4).

Each model is a thin vertical panel textured with a real tag36h11 marker (a white
quiet-zone border is added so apriltag_ros detects it reliably). The IDs and the
side length match rescue_bringup/config/apriltag_tags.yaml (victim_0..4, 16 cm).

Run once (no ROS / no Gazebo needed):

    python3 scripts/generate_apriltag_models.py

Outputs under ros2_ws/src/rescue_world/models/apriltag_36h11_0<ID>/ :
    model.config
    model.sdf
    materials/textures/tag36_11_0000<ID>.png

Then they can be <include>d in a world, or spawned with `ros2 run ros_gz_sim create`.
"""
from __future__ import annotations

import pathlib

import cv2
import numpy as np

# tag side length in metres — keep in sync with apriltag_tags.yaml (size: 0.16)
TAG_SIZE_M = 0.16
TAG_IDS = [0, 1, 2, 3, 4]
# texture resolution: marker rendered big, then a white quiet zone added around it
MARKER_PX = 720
QUIET_FRAC = 0.18  # white border = 18 % of the marker size on each side

HERE = pathlib.Path(__file__).resolve().parent
MODELS_DIR = HERE.parent / "ros2_ws" / "src" / "rescue_world" / "models"

MODEL_CONFIG = """<?xml version="1.0"?>
<model>
  <name>apriltag_36h11_{id:02d}</name>
  <version>1.0</version>
  <sdf version="1.8">model.sdf</sdf>
  <description>AprilTag tag36h11 id {id} (victim_{id}) — 16 cm panel for IA712 rescue.</description>
</model>
"""

# Thin vertical panel in the X-Z plane (thin along Y), so the tag faces +/-Y and a
# robot driving past sees it. PBR albedo_map carries the tag texture.
MODEL_SDF = """<?xml version="1.0"?>
<sdf version="1.8">
  <model name="apriltag_36h11_{id:02d}">
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <geometry>
          <box><size>{size} 0.008 {size}</size></box>
        </geometry>
        <material>
          <ambient>1 1 1 1</ambient>
          <diffuse>1 1 1 1</diffuse>
          <specular>0 0 0 1</specular>
          <pbr>
            <metal>
              <albedo_map>materials/textures/tag36_11_{id:05d}.png</albedo_map>
              <metalness>0.0</metalness>
              <roughness>1.0</roughness>
            </metal>
          </pbr>
        </material>
      </visual>
      <collision name="collision">
        <geometry>
          <box><size>{size} 0.008 {size}</size></box>
        </geometry>
      </collision>
    </link>
  </model>
</sdf>
"""


def make_texture(tag_id: int) -> np.ndarray:
    """tag36h11 marker with a white quiet-zone border, as a BGR image."""
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    marker = cv2.aruco.generateImageMarker(dictionary, tag_id, MARKER_PX)  # 8-bit, 1 ch
    border = int(MARKER_PX * QUIET_FRAC)
    canvas = np.full(
        (MARKER_PX + 2 * border, MARKER_PX + 2 * border), 255, dtype=np.uint8
    )
    canvas[border:border + MARKER_PX, border:border + MARKER_PX] = marker
    return cv2.cvtColor(canvas, cv2.COLOR_GRAY2BGR)


def main() -> None:
    for tag_id in TAG_IDS:
        model_name = f"apriltag_36h11_{tag_id:02d}"
        model_dir = MODELS_DIR / model_name
        tex_dir = model_dir / "materials" / "textures"
        tex_dir.mkdir(parents=True, exist_ok=True)

        tex_path = tex_dir / f"tag36_11_{tag_id:05d}.png"
        cv2.imwrite(str(tex_path), make_texture(tag_id))

        (model_dir / "model.config").write_text(MODEL_CONFIG.format(id=tag_id))
        (model_dir / "model.sdf").write_text(
            MODEL_SDF.format(id=tag_id, size=TAG_SIZE_M)
        )
        print(f"  {model_name}: texture {tex_path.name} + model.sdf/model.config")

    print(f"Done — {len(TAG_IDS)} AprilTag models under {MODELS_DIR}")


if __name__ == "__main__":
    main()
