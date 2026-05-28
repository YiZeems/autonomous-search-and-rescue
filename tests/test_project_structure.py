from __future__ import annotations

import ast
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PKG = ROOT / "ros2_ws" / "src" / "ia712_search_rescue"
PY_PKG = PKG / "ia712_search_rescue"


REQUIRED_PATHS = [
    ROOT / "README.md",
    ROOT / "CONTRIBUTING.md",
    ROOT / "AGENTS.md",
    ROOT / "docs" / "INSTALLATION.md",
    ROOT / "pyproject.toml",
    ROOT / "scripts" / "run.sh",
    PKG / "package.xml",
    PKG / "setup.py",
    PKG / "launch" / "bringup.launch.py",
    PKG / "launch" / "mock_system.launch.py",
    PKG / "behavior_trees" / "search_and_rescue_bt.xml",
    PKG / "config" / "explorer_params.yaml",
    PKG / "config" / "detector_params.yaml",
    PKG / "config" / "results_params.yaml",
    PKG / "config" / "bt_params.yaml",
]


ENTRYPOINT_MODULES = [
    PY_PKG / "exploration" / "frontier_explorer_node.py",
    PY_PKG / "detection" / "victim_detector_node.py",
    PY_PKG / "results" / "coverage_evaluator_node.py",
    PY_PKG / "results" / "result_exporter_node.py",
    PY_PKG / "results" / "rviz_marker_node.py",
    PY_PKG / "mocks" / "mock_map_publisher.py",
    PY_PKG / "mocks" / "mock_victim_publisher.py",
    PY_PKG / "mocks" / "mock_coverage_publisher.py",
    PY_PKG / "bt" / "bt_supervisor_node.py",
]


def test_required_files_exist() -> None:
    missing = [str(path.relative_to(ROOT)) for path in REQUIRED_PATHS if not path.exists()]
    assert not missing, "Missing required files: " + ", ".join(missing)


def test_entrypoint_modules_exist() -> None:
    missing = [str(path.relative_to(ROOT)) for path in ENTRYPOINT_MODULES if not path.exists()]
    assert not missing, "Missing entrypoint modules: " + ", ".join(missing)


def test_python_files_parse_without_generating_cache() -> None:
    python_files = [
        path
        for path in ROOT.rglob("*.py")
        if "build" not in path.parts and "install" not in path.parts and "log" not in path.parts
    ]
    for path in python_files:
        ast.parse(path.read_text(encoding="utf-8"), filename=str(path))


def test_no_ros_build_artifacts_in_repository() -> None:
    # Check that ROS build artifacts are not committed to git.
    # Uses git ls-files so that locally-generated but gitignored folders (build/, install/, log/)
    # don't cause false positives during development.
    import subprocess

    result = subprocess.run(
        ["git", "ls-files", "ros2_ws/build", "ros2_ws/install", "ros2_ws/log"],
        cwd=str(ROOT),
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        # Not a git repository — fall back to filesystem check only for the top-level dirs
        forbidden_names = {"build", "install", "log"}
        top_level_offenders = [
            f"ros2_ws/{name}"
            for name in forbidden_names
            if (ROOT / "ros2_ws" / name).exists()
        ]
        # In a non-git context (e.g. extracted archive), skip the check if .gitignore is present
        gitignore = ROOT / ".gitignore"
        if gitignore.exists():
            return  # .gitignore exists, assume artifacts are properly excluded
        assert not top_level_offenders, (
            "ROS generated artifacts should not be committed: " + ", ".join(top_level_offenders)
        )
        return

    committed = [line for line in result.stdout.splitlines() if line.strip()]
    assert not committed, (
        "ROS generated artifacts should not be committed to git: " + ", ".join(committed)
    )


def test_run_sh_references_existing_scripts() -> None:
    run_sh = ROOT / "scripts" / "run.sh"
    content = run_sh.read_text(encoding="utf-8")
    expected = [
        "install_apt_dependencies.sh",
        "build.sh",
        "clean.sh",
        "run_mock.sh",
        "run_bringup.sh",
        "run_simulation.sh",
        "run_navigation.sh",
        "run_teleop.sh",
        "run_exploration.sh",
        "run_detection.sh",
        "run_results.sh",
        "run_bt.sh",
        "uv_sync.sh",
        "uv_test.sh",
        "uv_lint.sh",
    ]
    for name in expected:
        if name in content:
            assert (ROOT / "scripts" / "sh" / name).exists(), f"{name} referenced but missing"


def test_simulation_world_scripts_exist():
    for script in [
        "scripts/sh/run_simulation_house.sh",
        "scripts/sh/run_simulation_base.sh",
        "scripts/sh/run_simulation_empty.sh",
    ]:
        assert Path(script).is_file(), script
