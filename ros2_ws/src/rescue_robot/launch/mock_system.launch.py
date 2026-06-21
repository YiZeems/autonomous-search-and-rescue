from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rescue_robot',
            executable='mock_map_publisher',
            name='mock_map_publisher',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='rescue_robot',
            executable='mock_victim_publisher',
            name='mock_victim_publisher',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='rescue_robot',
            executable='mock_coverage_publisher',
            name='mock_coverage_publisher',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='rescue_robot',
            executable='rviz_marker_node',
            name='rviz_marker_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
        Node(
            package='rescue_robot',
            executable='result_exporter_node',
            name='result_exporter_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
    ])
