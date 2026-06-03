# shellcheck shell=bash
# Platform profile: Windows 11 + WSL2, Ubuntu 22.04 x86_64.
#
# Sourced by scripts/sh/run_demo_tb4.sh. WSLg exposes the GPU to Linux through
# the Direct3D12 Mesa driver (/dev/dxg), so OpenGL is HARDWARE-accelerated —
# unlike the Parallels ARM64 case. That means:
#   - Ogre2 (Ignition's default) works, including the gpu_lidar depth pass, so
#     the RPLIDAR returns real ranges with no render-engine override needed.
#   - The Gazebo GUI opens normally through WSLg (no Ogre-v1 fallback).
#   - No software-GL forcing (that would DISABLE the WSLg GPU and re-introduce
#     the gpu_lidar range_min bug — see docs/ERRORS_AND_FIXES.md #10).
#
# If a teammate's WSLg GL turns out to be software-only (older Windows/driver),
# set IA712_RENDER_ENGINE=ogre and LIBGL_ALWAYS_SOFTWARE=1 to use the Mac path.

export IA712_PLATFORM_NAME="win-wsl2-x86_64"

# --- Render engine: Ogre2 default (hardware GPU via WSLg) ---
export IA712_RENDER_ENGINE="${IA712_RENDER_ENGINE:-ogre2}"

# --- Do NOT force software GL: use the WSLg D3D12 GPU ---
unset LIBGL_ALWAYS_SOFTWARE
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"
export DISPLAY="${DISPLAY:-:0}"

# --- DDS: default transport is fine on WSL2; clean stale SHM defensively ---
export IA712_USE_UDP_DDS="${IA712_USE_UDP_DDS:-0}"
export IA712_CLEAN_SHM=1

# --- GUI: on by default (normal Ogre2 window through WSLg) ---
export IA712_TB4_GUI="${IA712_TB4_GUI:-1}"
