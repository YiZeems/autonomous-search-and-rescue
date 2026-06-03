"""Tests for the Mac/Parallels vs Windows/WSL2 platform split.

These pin the configuration that makes the TurtleBot4 + Ignition lidar work:
  - Mac/Parallels (ARM64, software GL) MUST use the Ogre v1 render engine,
    otherwise gpu_lidar returns range_min on every ray (ERRORS_AND_FIXES #10).
  - WSL2 (x86, hardware WSLg GPU) uses Ogre2 and must NOT force software GL.
  - run_demo_tb4.sh must wire the profile + scan throttle, or the SLAM map
    stays frozen at 7x7 (ERRORS_AND_FIXES #25).
"""
from __future__ import annotations

import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CONFIG = ROOT / "config"
SH = ROOT / "scripts" / "sh"
PY_PKG = ROOT / "ros2_ws" / "src" / "rescue_robot" / "rescue_robot"


# ---------------------------------------------------------------------------
# Platform profiles
# ---------------------------------------------------------------------------

def test_platform_profiles_exist() -> None:
    assert (CONFIG / "platform_mac.sh").exists(), "config/platform_mac.sh missing"
    assert (CONFIG / "platform_win.sh").exists(), "config/platform_win.sh missing"
    assert (SH / "_platform.sh").exists(), "scripts/sh/_platform.sh missing"


def test_mac_profile_uses_ogre_v1() -> None:
    """Mac/Parallels must use Ogre v1 — Ogre2 gpu_lidar fails under software GL."""
    txt = (CONFIG / "platform_mac.sh").read_text()
    m = re.search(r'IA712_RENDER_ENGINE=["\']?(\w+)', txt)
    assert m and m.group(1) == "ogre", (
        "platform_mac.sh must set IA712_RENDER_ENGINE=ogre (Ogre v1), "
        f"found: {m.group(1) if m else None}"
    )
    assert "LIBGL_ALWAYS_SOFTWARE" in txt, "mac profile must force software GL"


def test_win_profile_uses_ogre2_and_no_software_gl() -> None:
    """WSL2 has a hardware GPU via WSLg — Ogre2 default, software GL must be off."""
    txt = (CONFIG / "platform_win.sh").read_text()
    assert "ogre2" in txt, "platform_win.sh should use the Ogre2 default engine"
    # Must NOT force software rendering (that re-breaks gpu_lidar)
    assert "unset LIBGL_ALWAYS_SOFTWARE" in txt or "LIBGL_ALWAYS_SOFTWARE=1" not in txt, (
        "platform_win.sh must not force LIBGL_ALWAYS_SOFTWARE=1 (disables WSLg GPU)"
    )


def test_platform_detector_handles_wsl_and_arm() -> None:
    txt = (SH / "_platform.sh").read_text()
    assert "microsoft" in txt.lower() or "wsl" in txt.lower(), (
        "_platform.sh must detect WSL via the kernel string"
    )
    assert "aarch64" in txt or "arm64" in txt, "_platform.sh must detect ARM64 (mac)"
    assert "IA712_PLATFORM" in txt, "_platform.sh must honour the IA712_PLATFORM override"


# ---------------------------------------------------------------------------
# run_demo_tb4.sh wiring
# ---------------------------------------------------------------------------

def test_demo_tb4_sources_platform_and_uses_render_engine() -> None:
    txt = (SH / "run_demo_tb4.sh").read_text()
    assert "_platform.sh" in txt, "run_demo_tb4.sh must source _platform.sh"
    assert "--render-engine" in txt, (
        "run_demo_tb4.sh must pass --render-engine to ign gazebo (server + GUI)"
    )
    assert "IA712_RENDER_ENGINE" in txt, "run_demo_tb4.sh must use the profile's render engine"


def test_demo_tb4_throttles_scan() -> None:
    """Lidar must be bridged to /scan_raw then throttled to /scan (#25)."""
    txt = (SH / "run_demo_tb4.sh").read_text()
    assert "/scan_raw" in txt, "lidar must be bridged to /scan_raw (then throttled)"
    assert "scan_throttle_node" in txt, "run_demo_tb4.sh must launch scan_throttle_node"


# ---------------------------------------------------------------------------
# scan_throttle_node
# ---------------------------------------------------------------------------

def test_scan_throttle_node_exists_with_main_and_node() -> None:
    src = (PY_PKG / "utils" / "scan_throttle_node.py").read_text()
    assert "def main(" in src, "scan_throttle_node must define main()"
    assert "class ScanThrottleNode(Node)" in src, "scan_throttle_node must subclass Node"
    # default in->out topics so SLAM (on /scan) keeps working unchanged
    assert "/scan_raw" in src and "/scan" in src


def test_scan_throttle_registered_as_entrypoint() -> None:
    setup_py = (ROOT / "ros2_ws" / "src" / "rescue_robot" / "setup.py").read_text()
    assert "scan_throttle_node = rescue_robot.utils.scan_throttle_node:main" in setup_py, (
        "scan_throttle_node entry point missing from setup.py"
    )


def test_throttle_rate_limit_logic() -> None:
    """Reproduce the node's gate: forward only when >= period since last publish."""
    period = 1.0 / 10.0  # 10 Hz
    last = 0.0
    forwarded = 0
    # simulate 100 incoming scans at 300 Hz (dt = 1/300 s)
    for i in range(100):
        now = i / 300.0
        if now - last >= period:
            last = now
            forwarded += 1
    # 100 scans over ~0.33 s at 10 Hz cap -> ~3-4 forwarded, never all 100
    assert 2 <= forwarded <= 5, f"throttle let through {forwarded} (expected ~3-4)"
