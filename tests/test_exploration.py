"""Tests for the autonomous frontier-exploration wiring (params + launch + demo).

The pure algorithm is covered by test_frontier_search.py; this file pins the
integration glue so the explorer actually runs in the demo.
"""
from __future__ import annotations

import ast
import re
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
PKG = ROOT / "ros2_ws" / "src" / "rescue_robot"
CONFIG = PKG / "config"
LAUNCH = PKG / "launch"
SH = ROOT / "scripts" / "sh"


# ---------------------------------------------------------------------------
# explorer_params.yaml
# ---------------------------------------------------------------------------

def _params() -> dict:
    data = yaml.safe_load((CONFIG / "explorer_params.yaml").read_text())
    return data["frontier_explorer_node"]["ros__parameters"]


def test_explorer_params_use_sim_time() -> None:
    assert _params().get("use_sim_time") is True, "explorer must use the sim clock"


def test_explorer_params_coverage_threshold() -> None:
    assert _params().get("coverage_stop_threshold") == 0.90, (
        "coverage_stop_threshold must be 0.90 (project success criterion)"
    )


def test_explorer_params_has_frames_and_timing() -> None:
    p = _params()
    for key in ("map_frame", "base_frame", "min_frontier_size", "replan_period_sec"):
        assert key in p, f"explorer_params.yaml missing '{key}'"
    assert p["min_frontier_size"] >= 1
    assert p["replan_period_sec"] > 0


def test_explorer_params_match_node_declarations() -> None:
    """Every param in the YAML must be declared by the node (no silent typos)."""
    src = (PKG / "rescue_robot" / "exploration" / "frontier_explorer_node.py").read_text()
    declared = set(re.findall(r'declare_parameter\(\s*["\'](\w+)["\']', src))
    declared.add("use_sim_time")  # provided by rclpy automatically
    for key in _params():
        assert key in declared, f"explorer_params.yaml has '{key}' but the node never declares it"


# ---------------------------------------------------------------------------
# exploration.launch.py
# ---------------------------------------------------------------------------

def test_exploration_launch_parses_and_runs_explorer() -> None:
    src = (LAUNCH / "exploration.launch.py").read_text()
    tree = ast.parse(src)
    assert any(
        isinstance(n, ast.FunctionDef) and n.name == "generate_launch_description"
        for n in ast.walk(tree)
    ), "exploration.launch.py missing generate_launch_description()"
    assert "frontier_explorer_node" in src, "launch must run frontier_explorer_node"
    assert "use_sim_time" in src and "base_frame" in src, (
        "launch must expose use_sim_time and base_frame arguments"
    )


# ---------------------------------------------------------------------------
# demo integration
# ---------------------------------------------------------------------------

def test_demo_tb4_supports_autonomous_exploration() -> None:
    txt = (SH / "run_demo_tb4.sh").read_text()
    assert "IA712_EXPLORE" in txt, "run_demo_tb4.sh must support IA712_EXPLORE mode"
    assert "exploration.launch.py" in txt, "demo must be able to launch exploration"
