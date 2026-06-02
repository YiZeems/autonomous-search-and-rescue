from __future__ import annotations

from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PKG = ROOT / "ros2_ws" / "src" / "rescue_robot"
BRINGUP = ROOT / "ros2_ws" / "src" / "rescue_bringup"
PY_PKG = PKG / "rescue_robot"


REQUIRED_PATHS = [
    ROOT / "README.md",
    ROOT / "CONTRIBUTING.md",
    ROOT / "AGENTS.md",
    ROOT / "docs" / "INSTALLATION.md",
    ROOT / "pyproject.toml",
    ROOT / "scripts" / "run.sh",
    # rescue_robot — nodes
    PKG / "package.xml",
    PKG / "setup.py",
    PKG / "launch" / "mock_system.launch.py",
    PKG / "behavior_trees" / "search_and_rescue_bt.xml",
    PKG / "config" / "explorer_params.yaml",
    PKG / "config" / "detector_params.yaml",
    PKG / "config" / "results_params.yaml",
    PKG / "config" / "bt_params.yaml",
    # rescue_bringup — main entry point
    BRINGUP / "package.xml",
    BRINGUP / "launch" / "bringup.launch.py",
    BRINGUP / "config" / "slam_params.yaml",
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


def test_all_scripts_are_executable() -> None:
    """Every .sh file referenced in run.sh must be executable (catches chmod issues)."""
    missing_x = []
    for script in (ROOT / "scripts" / "sh").glob("*.sh"):
        if not script.stat().st_mode & 0o111:
            missing_x.append(str(script.relative_to(ROOT)))
    assert not missing_x, "Scripts not executable: " + ", ".join(missing_x)


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
    """Every .sh file referenced in run.sh must actually exist on disk."""
    run_sh = ROOT / "scripts" / "run.sh"
    content = run_sh.read_text(encoding="utf-8")
    # All scripts referenced, including newer demo/waypoint/rviz ones
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
        "run_demo.sh",
        "run_demo_tb4.sh",
        "run_waypoint.sh",
        "run_rviz.sh",
        "uv_sync.sh",
        "uv_test.sh",
        "uv_lint.sh",
    ]
    for name in expected:
        assert name in content, f"{name} missing from run.sh — command won't work"
        assert (ROOT / "scripts" / "sh" / name).exists(), (
            f"{name} referenced in run.sh but file not found"
        )


def test_run_sh_all_cases_have_scripts() -> None:
    """Parse run.sh case block — every case must have a .sh file."""
    import re
    run_sh = (ROOT / "scripts" / "run.sh").read_text()
    # Extract lines like: demo) "...run_demo.sh" or demo-tb4) "...run_demo_tb4.sh"
    matches = re.findall(r'"[^"]*/([\w_]+\.sh)"', run_sh)
    for script_name in matches:
        assert (ROOT / "scripts" / "sh" / script_name).exists(), (
            f"run.sh references {script_name} but file not found in scripts/sh/"
        )


def test_demo_scripts_have_cleanup_trap() -> None:
    """Both demo scripts must have a Ctrl+C cleanup trap to avoid zombie processes.
    This was a real bug: Ctrl+C left gzserver/controller_server running."""
    for script_name in ("run_demo.sh", "run_demo_tb4.sh"):
        content = (ROOT / "scripts" / "sh" / script_name).read_text()
        assert "trap _cleanup" in content, (
            f"{script_name}: missing cleanup trap — Ctrl+C will leave zombie processes"
        )
        assert "pkill" in content, (
            f"{script_name}: cleanup must pkill known processes (gzserver, slam_toolbox…)"
        )


def test_bringup_launch_uses_project_rviz_config() -> None:
    """bringup.launch.py must use the project's project_view.rviz, not nav2's default.

    nav2_default_view.rviz has no camera feed.  The project config includes the
    Image display for /camera/image_raw and the victim-marker overlay.
    """
    content = (BRINGUP / "launch" / "bringup.launch.py").read_text()
    assert "nav2_default_view.rviz" not in content, (
        "bringup.launch.py must not use nav2_default_view.rviz — "
        "it lacks camera feed and victim markers. Use project_view.rviz instead."
    )
    assert "project_view.rviz" in content, (
        "bringup.launch.py must reference project_view.rviz"
    )


def test_rviz_config_file_exists_for_all_launches() -> None:
    """All launch files that reference project_view.rviz must point to an existing file."""
    rviz_config = PKG / "rviz" / "project_view.rviz"
    assert rviz_config.exists(), "rviz/project_view.rviz does not exist"
    for launch_name in ("demo.launch.py", "bringup.launch.py"):
        content = (BRINGUP / "launch" / launch_name).read_text()
        assert "project_view.rviz" in content, (
            f"rescue_bringup/launch/{launch_name}: must reference project_view.rviz"
        )
