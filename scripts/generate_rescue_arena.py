#!/usr/bin/env python3
"""Generate a search-and-rescue Ignition Gazebo world (rescue_arena.sdf).

Design goals for the IA712 project:
  - 12x12 m "collapsed building": 4 rooms + a cross corridor with doorways, so
    the robot has somewhere to explore and SLAM has real walls to map.
  - Built from box primitives + static AprilTag model includes — no external
    meshes. This renders correctly under software GL (llvmpipe) AND is detected
    by the gpu_lidar, unlike mesh-heavy worlds (see docs/ERRORS_AND_FIXES.md #10).
  - Origin (0,0) is kept clear (central corridor) so the robot never spawns
    embedded in a wall.
  - Scattered "rubble" boxes (obstacles) + 4 AprilTag victims (one per room),
    placed on the outer walls at OAK-D camera height so apriltag_ros detects
    them. This world is the single source of truth for victim placement;
    generate_apriltag_models.py only builds the tag model assets it references.
  - physics block + shadows OFF (software-renderer stability).

Run:  python3 scripts/generate_rescue_arena.py
Out:  ros2_ws/src/rescue_world/worlds/rescue_arena.sdf
"""
from __future__ import annotations

import os
from pathlib import Path

WALL_T = 0.15      # wall thickness (m)
WALL_H = 1.0       # wall height (m)
A = 6.0            # half-arena size (m) -> 12x12 m

# Axis-aligned wall segments as (x1, y1, x2, y2). Gaps between segments are
# doorways. The band y in [-2, 2] is left open as the main corridor (origin).
WALLS = [
    # --- outer walls ---
    (-A,  A,  A,  A),     # north
    (-A, -A,  A, -A),     # south
    ( A, -A,  A,  A),     # east
    (-A, -A, -A,  A),     # west
    # --- divider at y=2 (north rooms / corridor), WIDE doorways near x=-3 and x=+3 ---
    (-A, 2.0, -3.9, 2.0),
    (-2.1, 2.0, 2.1, 2.0),
    (3.9, 2.0, A, 2.0),
    # --- divider at y=-2 (south rooms / corridor), doorways near x=-0.5 and x=+0.5 ---
    (-A, -2.0, -1.0, -2.0),
    (1.0, -2.0, A, -2.0),
    # --- vertical split of the north band (x=0), WIDE doorway at y in [3.2, 4.8] ---
    (0.0, 2.0, 0.0, 3.2),
    (0.0, 4.8, 0.0, A),
    # --- vertical split of the south band (x=0), WIDE doorway at y in [-4.8, -3.2] ---
    (0.0, -A, 0.0, -4.8),
    (0.0, -3.2, 0.0, -2.0),
]

# Rubble obstacles: (x, y, size) cubes.
RUBBLE = [
    (3.6, 4.2, 0.45),
    (-3.8, 4.0, 0.55),
    (-3.6, -4.2, 0.35),
    (3.9, -3.8, 0.4),
    (-4.6, 0.0, 0.3),
]

# AprilTag victims: (id, x, y, z, roll_deg, pitch_deg, yaw_deg), one per room.
# Poses MUST match scripts/generate_apriltag_models.py (which only builds the
# tag model assets these <include> blocks reference).
# roll=+90 -> tag faces south (mounted on a north wall, y>0);
# roll=-90 -> tag faces north (mounted on a south wall, y<0).
VICTIM_TAGS = [
    (0,  4.6,  5.924, 0.28,  90, 0, 0),   # NE room, north wall, facing south
    (1, -4.6, -5.924, 0.28, -90, 0, 0),   # SW room, south wall, facing north
    (2, -4.6,  5.924, 0.28,  90, 0, 0),   # NW room, north wall, facing south
    (3,  4.6, -5.924, 0.28, -90, 0, 0),   # SE room, south wall, facing north
]


def _box_link(name: str, cx: float, cy: float, cz: float,
              sx: float, sy: float, sz: float, rgba: str) -> str:
    return f"""
        <model name='{name}'>
            <static>1</static>
            <pose>{cx:.3f} {cy:.3f} {cz:.3f} 0 0 0</pose>
            <link name='link'>
                <collision name='collision'>
                    <geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>
                </collision>
                <visual name='visual'>
                    <geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>
                    <material>
                        <ambient>{rgba}</ambient><diffuse>{rgba}</diffuse>
                    </material>
                </visual>
            </link>
        </model>"""


def _tag_include(tag_id: int, name: str, x: float, y: float, z: float,
                 roll_deg: float, pitch_deg: float, yaw_deg: float) -> str:
    import math

    r = math.radians(roll_deg)
    p = math.radians(pitch_deg)
    yw = math.radians(yaw_deg)
    return f"""
        <include>
            <uri>model://apriltag_36h11_{tag_id:02d}</uri>
            <name>{name}</name>
            <pose>{x:.4f} {y:.4f} {z:.4f} {r:.4f} {p:.4f} {yw:.4f}</pose>
        </include>"""


def _wall_model(idx: int, x1: float, y1: float, x2: float, y2: float) -> str:
    cx, cy = (x1 + x2) / 2.0, (y1 + y2) / 2.0
    sx = abs(x2 - x1) or WALL_T
    sy = abs(y2 - y1) or WALL_T
    # extend by thickness so corners overlap cleanly
    if sx > WALL_T:
        sx += WALL_T
    if sy > WALL_T:
        sy += WALL_T
    return _box_link(f"wall_{idx}", cx, cy, WALL_H / 2.0, sx, sy, WALL_H, "0.5 0.5 0.55 1")


def build_world() -> str:
    parts = []
    for i, (x1, y1, x2, y2) in enumerate(WALLS):
        parts.append(_wall_model(i, x1, y1, x2, y2))
    for i, (x, y, s) in enumerate(RUBBLE):
        parts.append(_box_link(f"rubble_{i}", x, y, s / 2.0, s, s, s, "0.45 0.32 0.2 1"))
    parts.append("\n        <!-- AprilTag victims (one per room) -->")
    for tag_id, x, y, z, roll, pitch, yaw in VICTIM_TAGS:
        parts.append(_tag_include(tag_id, f"victim_{tag_id}", x, y, z, roll, pitch, yaw))
    bodies = "".join(parts)
    # Gazebo update rate (Hz). With max_step_size=0.01 this sets the RTF:
    # 100 -> RTF 1.0 (realtime); lower -> fewer physics + sensor (camera!) renders
    # per wall-second = less sustained GPU/VRAM/CPU load on WSL, which mitigates the
    # 0x10e VIDEO_MEMORY_MANAGEMENT_INTERNAL GPU crash and thermal reboots — at the
    # cost of slower-than-realtime sim. Override with IA712_GZ_RT_RATE=100 for speed.
    rt_rate = os.environ.get("IA712_GZ_RT_RATE", "70")
    return f"""<?xml version="1.0"?>
<!-- Auto-generated by scripts/generate_rescue_arena.py — do not edit by hand. -->
<sdf version='1.8'>
    <world name='rescue_arena'>
        <physics name='1ms' type='ignored'>
            <!-- Coarser step (0.01 vs 0.001 default) = ~3x fewer physics iterations.
                 Safe for the slow diff-drive TB4; Nav2/SLAM don't need fine physics.
                 real_time_update_rate is the GPU/CPU-load knob (see rt_rate above):
                 70 -> RTF 0.7, lighter sustained load; IA712_GZ_RT_RATE=100 for realtime. -->
            <max_step_size>0.01</max_step_size>
            <real_time_factor>1</real_time_factor>
            <real_time_update_rate>{rt_rate}</real_time_update_rate>
        </physics>
        <plugin name='ignition::gazebo::systems::Physics' filename='ignition-gazebo-physics-system' />
        <plugin name='ignition::gazebo::systems::UserCommands' filename='ignition-gazebo-user-commands-system' />
        <plugin name='ignition::gazebo::systems::SceneBroadcaster' filename='ignition-gazebo-scene-broadcaster-system' />
        <plugin name='ignition::gazebo::systems::Contact' filename='ignition-gazebo-contact-system' />
        <light name='sun' type='directional'>
            <cast_shadows>0</cast_shadows>
            <pose>0 0 10 0 0 0</pose>
            <diffuse>0.9 0.9 0.9 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
            <attenuation><range>1000</range><constant>0.9</constant><linear>0.01</linear><quadratic>0.001</quadratic></attenuation>
            <direction>-0.4 0.2 -0.9</direction>
        </light>
        <gravity>0 0 -9.8</gravity>
        <scene>
            <ambient>0.5 0.5 0.5 1</ambient>
            <background>0.7 0.7 0.75 1</background>
            <shadows>0</shadows>
        </scene>
        <model name='ground_plane'>
            <static>1</static>
            <link name='link'>
                <collision name='collision'>
                    <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
                </collision>
                <visual name='visual'>
                    <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
                    <material><ambient>0.3 0.3 0.3 1</ambient><diffuse>0.35 0.35 0.35 1</diffuse></material>
                </visual>
            </link>
        </model>{bodies}
    </world>
</sdf>
"""


def main() -> None:
    out = (
        Path(__file__).resolve().parents[1]
        / "ros2_ws" / "src" / "rescue_world" / "worlds" / "rescue_arena.sdf"
    )
    out.write_text(build_world(), encoding="utf-8")
    print(f"wrote {out}  ({len(WALLS)} walls, {len(RUBBLE)} rubble, {len(VICTIM_TAGS)} victims)")


if __name__ == "__main__":
    main()
