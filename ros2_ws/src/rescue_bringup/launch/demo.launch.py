"""Full demo launch — starts everything in ONE command, no separate terminals needed.

Launches in order:
  1. Gazebo (gzserver + gzclient) with turtlebot3_house
  2. TurtleBot3 Waffle Pi (robot_state_publisher + spawn)
  3. SLAM Toolbox (online async)
  4. Nav2 (navigation servers)
  5. RViz2
  6. Waypoint follower (waits 15s for Nav2 to be ready, then starts the path)

Usage:
    # Full demo with default house world + predefined path
    ./scripts/run.sh demo

    # Headless (no Gazebo GUI, useful for CI or low-resource runs)
    ./scripts/run.sh demo headless

    # Custom waypoints
    ros2 launch rescue_robot full_demo.launch.py \\
        waypoints_file:=/abs/path/to/my_waypoints.yaml

    # Loop the path indefinitely
    ros2 launch rescue_robot full_demo.launch.py loop:=true

    # Safe graphics mode (Parallels ARM64 / WSLg)
    IA712_GAZEBO_SAFE_GRAPHICS=1 ./scripts/run.sh demo
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# TURTLEBOT3_MODEL must be set before turtlebot3_gazebo launch files are imported
os.environ.setdefault("TURTLEBOT3_MODEL", "waffle_pi")


def generate_launch_description():
    pkg_ia712 = FindPackageShare("rescue_robot")
    pkg_tb3_gazebo = FindPackageShare("turtlebot3_gazebo")
    pkg_slam = FindPackageShare("slam_toolbox")
    pkg_nav2 = FindPackageShare("nav2_bringup")
    pkg_gazebo_ros = FindPackageShare("gazebo_ros")

    use_sim_time = LaunchConfiguration("use_sim_time")
    headless = LaunchConfiguration("headless")
    launch_rviz = LaunchConfiguration("launch_rviz")
    loop = LaunchConfiguration("loop")
    waypoints_file = LaunchConfiguration("waypoints_file")
    nav2_ready_delay = LaunchConfiguration("nav2_ready_delay")

    try:
        default_world = os.path.join(
            get_package_share_directory("turtlebot3_gazebo"),
            "worlds", "turtlebot3_house.world",
        )
    except Exception:
        default_world = ""

    # --- Gazebo (split server/client so headless works) ---
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_gazebo_ros, "launch", "gzserver.launch.py"])
        ),
        launch_arguments={"world": default_world}.items(),
    )
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_gazebo_ros, "launch", "gzclient.launch.py"])
        ),
        condition=UnlessCondition(headless),
    )

    # --- TurtleBot3 ---
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_tb3_gazebo, "launch", "robot_state_publisher.launch.py"])
        ),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )
    spawn_tb3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_tb3_gazebo, "launch", "spawn_turtlebot3.launch.py"])
        ),
        launch_arguments={"x_pose": "-2.0", "y_pose": "-0.5"}.items(),
    )

    # --- SLAM Toolbox ---
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_slam, "launch", "online_async_launch.py"])
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "slam_params_file": PathJoinSubstitution([pkg_ia712, "config", "slam_params.yaml"]),
        }.items(),
    )

    # --- Nav2 ---
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_nav2, "launch", "navigation_launch.py"])
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": PathJoinSubstitution([pkg_ia712, "config", "nav2_params.yaml"]),
        }.items(),
    )

    # --- RViz2 ---
    rviz_config = PathJoinSubstitution([pkg_ia712, "rviz", "project_view.rviz"])
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(launch_rviz),
        output="log",
    )

    # --- Waypoint follower (delayed: Nav2 needs time to start up) ---
    waypoint_follower = TimerAction(
        period=nav2_ready_delay,
        actions=[
            Node(
                package="rescue_robot",
                executable="waypoint_follower_node",
                name="waypoint_follower_node",
                output="screen",
                parameters=[{
                    "waypoints_file": waypoints_file,
                    "loop": loop,
                    "goal_timeout_sec": 60.0,
                    "use_sim_time": use_sim_time,
                }],
            )
        ],
    )

    return LaunchDescription([
        # --- Args ---
        DeclareLaunchArgument("use_sim_time", default_value="true",
                              description="Use Gazebo /clock."),
        DeclareLaunchArgument("headless", default_value="false",
                              description="No Gazebo GUI (CI/benchmark mode)."),
        DeclareLaunchArgument("launch_rviz", default_value="true",
                              description="Launch RViz2."),
        DeclareLaunchArgument(
            "waypoints_file",
            default_value=PathJoinSubstitution([pkg_ia712, "config", "waypoints.yaml"]),
            description="Path to waypoints YAML. Default: config/waypoints.yaml",
        ),
        DeclareLaunchArgument("loop", default_value="false",
                              description="Loop the waypoint path indefinitely."),
        DeclareLaunchArgument(
            "nav2_ready_delay",
            default_value="15.0",
            description="Seconds to wait for Nav2 before starting waypoint follower.",
        ),

        # --- Launch sequence ---
        gzserver,
        gzclient,
        robot_state_publisher,
        spawn_tb3,
        slam,
        nav2,
        rviz,
        waypoint_follower,
    ])
