"""Victim registry — AprilTag detections projected to the map frame.

Requires apriltag_ros running (bringup_tb4.launch.py launches it on the
OAK-D stream) and the TF chain map -> camera -> victim_<id> available.

    ros2 launch rescue_robot victim_registry.launch.py
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use the simulation clock.'),
        DeclareLaunchArgument('map_frame', default_value='map',
                              description='Global frame victims are registered in.'),
        Node(
            package='rescue_robot',
            executable='victim_registry_node',
            name='victim_registry_node',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'map_frame': LaunchConfiguration('map_frame'),
            }],
        ),
    ])
