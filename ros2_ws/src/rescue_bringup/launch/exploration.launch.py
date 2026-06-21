"""Exploration launch — runs the frontier-based exploration node on top of
an already-running bringup (Gazebo + SLAM + Nav2).

L15: wraps the vendored `explore_lite` (from m-explore-ros2) so the user can
start exploration in a second terminal after `bringup.launch.py` has settled:

    # Terminal 1
    ros2 launch rescue_bringup bringup.launch.py
    # Terminal 2 (once /map and Nav2 are up)
    ros2 launch rescue_bringup exploration.launch.py

`algo:=info_gain` selects our custom information-gain node from rescue_robot.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration('use_sim_time')
    algo = LaunchConfiguration('algo')

    # m-explore-ros2 / explore_lite ships its own launch file at
    # explore_lite/launch/explore.launch.py — reuse it.
    explore_lite = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('explore_lite'), 'launch', 'explore.launch.py'])
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use Gazebo /clock.'),
        DeclareLaunchArgument('algo', default_value='greedy',
                              choices=['greedy', 'info_gain'],
                              description='Exploration algorithm (greedy | info_gain).'),
        LogInfo(msg=['[rescue_bringup] exploration starting — algo=', algo]),
        explore_lite,
    ])
