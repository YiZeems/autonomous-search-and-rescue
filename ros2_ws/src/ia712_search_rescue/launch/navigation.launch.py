import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch SLAM Toolbox (online async) + Nav2 navigation stack.

    SLAM Toolbox builds /map from /scan in real time.
    Nav2 handles path planning and goal execution (/cmd_vel).
    This launch file is included by bringup.launch.py after simulation starts.
    """

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true.",
    )

    # --- SLAM Toolbox (online async mode) ---
    slam_params_file = PathJoinSubstitution(
        [FindPackageShare("ia712_search_rescue"), "config", "slam_params.yaml"]
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

    # --- Nav2 (navigation stack, no AMCL — we use SLAM for localisation) ---
    nav2_params_file = PathJoinSubstitution(
        [FindPackageShare("ia712_search_rescue"), "config", "nav2_params.yaml"]
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
            LogInfo(msg="Starting SLAM Toolbox (online async) + Nav2 navigation stack."),
            slam_toolbox_launch,
            nav2_launch,
        ]
    )
