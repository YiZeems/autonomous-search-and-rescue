#!/usr/bin/env bash
# Helper sourced by simulation scripts.
# Purpose: make Gazebo Classic more stable in virtualized or fragile graphics
# environments such as Parallels ARM64, WSLg, or Wayland sessions.
set -eo pipefail

apply_gazebo_safe_graphics_if_requested() {
  # Auto-enable safe graphics on Parallels ARM64 / Wayland sessions to avoid
  # Gazebo and RViz2 freezes ("Ignoring XDG_SESSION_TYPE=wayland").
  if [ "${XDG_SESSION_TYPE:-}" = "wayland" ] || [ "${WAYLAND_DISPLAY:-}" != "" ]; then
    if [ "${IA712_GAZEBO_SAFE_GRAPHICS:-0}" != "1" ]; then
      echo "[INFO] Wayland session detected. Auto-enabling safe graphics mode."
      export IA712_GAZEBO_SAFE_GRAPHICS=1
    fi
  fi

  if [ "${IA712_GAZEBO_SAFE_GRAPHICS:-0}" = "1" ]; then
    echo "[INFO] Enabling Gazebo safe graphics mode."
    echo "[INFO] Setting QT_QPA_PLATFORM=xcb, GDK_BACKEND=x11, LIBGL_ALWAYS_SOFTWARE=1."
    export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"
    export GDK_BACKEND="${GDK_BACKEND:-x11}"
    export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
    export MESA_GL_VERSION_OVERRIDE="${MESA_GL_VERSION_OVERRIDE:-3.3}"
    export MESA_GLSL_VERSION_OVERRIDE="${MESA_GLSL_VERSION_OVERRIDE:-330}"
  fi
}
