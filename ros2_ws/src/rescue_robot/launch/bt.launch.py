from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params = PathJoinSubstitution([
        FindPackageShare('rescue_robot'),
        'config',
        'bt_params.yaml',
    ])
    return LaunchDescription([
        Node(
            package='rescue_robot',
            executable='bt_supervisor_node',
            name='bt_supervisor_node',
            output='screen',
            parameters=[params, {'use_sim_time': True}],
        ),
    ])
