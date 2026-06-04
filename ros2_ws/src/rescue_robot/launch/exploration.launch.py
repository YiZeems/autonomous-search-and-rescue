"""Autonomous frontier exploration.

Launches frontier_explorer_node, which drives the robot to map frontiers via
Nav2 (NavigateToPose) until coverage_stop_threshold is reached. Requires SLAM +
Nav2 already running (e.g. navigation.launch.py / navigation_tb4.launch.py).

    # TurtleBot3 (Gazebo Classic)
    ros2 launch rescue_robot exploration.launch.py

    # TurtleBot4 (Ignition) — robot frame is namespaced
    ros2 launch rescue_robot exploration.launch.py base_frame:=turtlebot4/base_link
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    base_frame = LaunchConfiguration("base_frame")

    params = PathJoinSubstitution([
        FindPackageShare("rescue_robot"), "config", "explorer_params.yaml",
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time", default_value="true",
            description="Use the simulation clock.",
        ),
        DeclareLaunchArgument(
            "base_frame", default_value="base_footprint",
            description="Robot base frame (TB4 Ignition: turtlebot4/base_link).",
        ),
        Node(
            package="rescue_robot",
            executable="frontier_explorer_node",
            name="frontier_explorer_node",
            output="screen",
            parameters=[
                params,
                {"use_sim_time": use_sim_time, "base_frame": base_frame},
            ],
        ),
    ])
