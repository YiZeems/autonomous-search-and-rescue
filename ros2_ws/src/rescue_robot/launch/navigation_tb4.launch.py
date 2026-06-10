"""SLAM + Nav2 for TurtleBot4 + Ignition Gazebo.

Differences vs navigation.launch.py:
  - Uses slam_params_tb4.yaml (odom_frame: turtlebot4/odom,
    base_frame: turtlebot4/base_link)
  - Uses nav2_params_tb4.yaml (matching TB4 namespaced frames)

Config files are resolved in this order (first found wins):
  1. slam_params_file / nav2_params_file launch arguments (absolute paths)
  2. IA712_SLAM_PARAMS / IA712_NAV2_PARAMS environment variables
  3. FindPackageShare("rescue_robot") lookup (requires workspace sourced)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _resolve_config(arg_name: str, env_var: str, pkg_relative: str) -> str:
    """Return config file path: launch arg > env var > package share."""
    # env var override (set by run_demo_tb4.sh on NFS/Parallels)
    env_path = os.environ.get(env_var, "")
    if env_path and os.path.isfile(env_path):
        return env_path
    try:
        return os.path.join(
            get_package_share_directory("rescue_robot"), pkg_relative
        )
    except Exception:
        return env_path


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation clock.",
    )

    slam_params_file = _resolve_config(
        "slam_params_file",
        "IA712_SLAM_PARAMS",
        os.path.join("config", "slam_params_tb4.yaml"),
    )

    nav2_params_file = _resolve_config(
        "nav2_params_file",
        "IA712_NAV2_PARAMS",
        os.path.join("config", "nav2_params_tb4.yaml"),
    )

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("slam_toolbox"),
                "launch",
                "online_async_launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "slam_params_file": slam_params_file,
        }.items(),
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("nav2_bringup"),
                "launch",
                "navigation_launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": nav2_params_file,
        }.items(),
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            LogInfo(msg="Starting SLAM (TB4 frames) + Nav2 navigation stack."),
            slam_toolbox_launch,
            nav2_launch,
        ]
    )
