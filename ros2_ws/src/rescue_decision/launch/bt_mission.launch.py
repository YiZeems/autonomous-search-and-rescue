"""Mission Behavior Tree (BehaviorTree.CPP v3) launch.

Run on top of an active stack (SLAM publishing /map, coverage_evaluator
publishing /coverage, victim_registry publishing /victims_map):

    ros2 launch rescue_decision bt_mission.launch.py
    # custom tree / faster ticking
    ros2 launch rescue_decision bt_mission.launch.py bt_xml:=/path/to/tree.xml tick_rate_hz:=2.0

Monitor live in Groot: mode "Monitor", ZMQ localhost:1666.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    default_xml = PathJoinSubstitution(
        [FindPackageShare('rescue_decision'), 'bt_xml', 'mission.xml']
    )
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use the simulation clock.'),
        DeclareLaunchArgument('bt_xml', default_value=default_xml,
                              description='Path to the mission BT XML.'),
        DeclareLaunchArgument('tick_rate_hz', default_value='1.0',
                              description='Tree tick frequency.'),
        DeclareLaunchArgument('groot_zmq', default_value='true',
                              description='Expose the tree to Groot Monitor (ZMQ 1666).'),
        Node(
            package='rescue_decision',
            executable='bt_runner',
            name='bt_runner',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'bt_xml': LaunchConfiguration('bt_xml'),
                'tick_rate_hz': LaunchConfiguration('tick_rate_hz'),
                'groot_zmq': LaunchConfiguration('groot_zmq'),
            }],
        ),
    ])
