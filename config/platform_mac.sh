# shellcheck shell=bash
# Platform profile: macOS + Parallels Desktop, Ubuntu 22.04 ARM64 (aarch64).
#
# Sourced by scripts/sh/run_demo_tb4.sh. Sets the environment that makes the
# TurtleBot4 + Ignition Fortress stack work on the Parallels virtual GPU
# (1ab8:0010), which has NO hardware 3D acceleration (Mesa llvmpipe only).
#
# Why each setting (see docs/ERRORS_AND_FIXES.md):
#   - Ogre v1 render engine: Ogre2's gpu_lidar depth pass fails under software
#     GL → the RPLIDAR returns range_min on every ray (#10). Ogre v1 renders
#     real ranges. This applies to BOTH the headless sensor server AND the GUI.
#   - LIBGL_ALWAYS_SOFTWARE / Mesa overrides: force the llvmpipe software path.
#   - Fast-DDS UDP-only + SHM cleanup: stale /dev/shm SHM segments make the
#     controller_manager service unreachable (#21).
#   - GUI: Ogre2 crashes (#1); the Ogre-v1 client window opens fine.

export IA712_PLATFORM_NAME="mac-parallels-arm64"

# --- Render engine (the lidar fix) ---
export IA712_RENDER_ENGINE="ogre"          # Ogre v1 (NOT ogre2) for server sensors + GUI

# --- Software GL (Parallels has no real GPU) ---
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
export MESA_GL_VERSION_OVERRIDE="${MESA_GL_VERSION_OVERRIDE:-3.3}"
export MESA_GLSL_VERSION_OVERRIDE="${MESA_GLSL_VERSION_OVERRIDE:-330}"
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"
export GDK_BACKEND="${GDK_BACKEND:-x11}"
export DISPLAY="${DISPLAY:-:0}"

# --- DDS transport (avoid stale SHM lockups) ---
export IA712_USE_UDP_DDS=1
export IA712_CLEAN_SHM=1

# --- GUI: on by default, Ogre v1 client window ---
export IA712_TB4_GUI="${IA712_TB4_GUI:-1}"
