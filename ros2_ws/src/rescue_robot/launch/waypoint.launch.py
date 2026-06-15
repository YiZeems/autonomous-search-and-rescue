"""Launch the waypoint follower — sends a predefined path to Nav2.

Requires Nav2 to be running (start simulation + navigation first).

Usage:
    # Default path (turtlebot3_house sweep, 8 waypoints)
    ros2 launch rescue_robot waypoint.launch.py

    # Custom waypoints file
    ros2 launch rescue_robot waypoint.launch.py \\
        waypoints_file:=/abs/path/to/waypoints.yaml

    # Loop indefinitely
    ros2 launch rescue_robot waypoint.launch.py loop:=true

    # One-shot script shortcut
    ./scripts/run.sh waypoint
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_waypoints = PathJoinSubstitution(
        [FindPackageShare("rescue_robot"), "config", "waypoints.yaml"]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "waypoints_file",
            default_value=default_waypoints,
            description="Absolute path to a waypoints YAML file.",
        ),
        DeclareLaunchArgument(
            "loop",
            default_value="false",
            description="Loop the waypoint sequence indefinitely.",
        ),
        DeclareLaunchArgument(
            "goal_timeout_sec",
            default_value="180.0",
            description="Seconds (WALL clock) to wait for each NavigateToPose goal. "
            "60 s trop court (15 s sim à RTF 0.25), 120 s aussi (patrouille 0/4) ; 180 s "
            "donne ~70 s sim pour traverser l'arène → v10 a fait 3 victimes. L'OOM des "
            "runs longs vient de la FUITE caméra Gazebo (réglée à la source : oakd 8 Hz), "
            "pas du timeout — donc on garde 180 s pour fiabiliser la détection.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use Gazebo simulation clock.",
        ),
        DeclareLaunchArgument(
            "dwell_sec",
            default_value="4.0",
            description="Seconds to pause at each dwell waypoint. The room-inspection "
            "phase passes 12 s so the in-place spin completes a full 360° turn.",
        ),
        DeclareLaunchArgument(
            "dwell_spin_speed",
            default_value="0.6",
            description="Rotate-in-place rate (rad/s) during the dwell so the camera "
            "sweeps the room walls; 0 = dwell without spinning.",
        ),
        Node(
            package="rescue_robot",
            executable="waypoint_follower_node",
            name="waypoint_follower_node",
            output="screen",
            parameters=[{
                "waypoints_file": LaunchConfiguration("waypoints_file"),
                "loop": LaunchConfiguration("loop"),
                "goal_timeout_sec": LaunchConfiguration("goal_timeout_sec"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "dwell_sec": LaunchConfiguration("dwell_sec"),
                "dwell_spin_speed": LaunchConfiguration("dwell_spin_speed"),
            }],
        ),
    ])
