from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params = PathJoinSubstitution([
        FindPackageShare('ia712_search_rescue'),
        'config',
        'explorer_params.yaml',
    ])
    return LaunchDescription([
        Node(
            package='ia712_search_rescue',
            executable='frontier_explorer_node',
            name='frontier_explorer_node',
            output='screen',
            parameters=[params],
        ),
    ])
