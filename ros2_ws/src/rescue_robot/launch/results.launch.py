from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params = PathJoinSubstitution([
        FindPackageShare('rescue_robot'),
        'config',
        'results_params.yaml',
    ])
    return LaunchDescription([
        Node(
            package='rescue_robot',
            executable='coverage_evaluator_node',
            name='coverage_evaluator_node',
            output='screen',
            parameters=[params, {'use_sim_time': True}],
        ),
        Node(
            package='rescue_robot',
            executable='rviz_marker_node',
            name='rviz_marker_node',
            output='screen',
            parameters=[params, {'use_sim_time': True}],
        ),
        Node(
            package='rescue_robot',
            executable='result_exporter_node',
            name='result_exporter_node',
            output='screen',
            parameters=[params, {'use_sim_time': True}],
        ),
    ])
