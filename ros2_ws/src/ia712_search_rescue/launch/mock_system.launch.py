from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ia712_search_rescue',
            executable='mock_map_publisher',
            name='mock_map_publisher',
            output='screen',
        ),
        Node(
            package='ia712_search_rescue',
            executable='mock_victim_publisher',
            name='mock_victim_publisher',
            output='screen',
        ),
        Node(
            package='ia712_search_rescue',
            executable='mock_coverage_publisher',
            name='mock_coverage_publisher',
            output='screen',
        ),
        Node(
            package='ia712_search_rescue',
            executable='rviz_marker_node',
            name='rviz_marker_node',
            output='screen',
        ),
        Node(
            package='ia712_search_rescue',
            executable='result_exporter_node',
            name='result_exporter_node',
            output='screen',
        ),
        Node(
            package='ia712_search_rescue',
            executable='bt_supervisor_node',
            name='bt_supervisor_node',
            output='screen',
        ),
    ])
