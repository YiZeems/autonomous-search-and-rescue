"""Single entry-point launch file — TurtleBot 4 stack (Ignition Gazebo Fortress).

L15 (TB4): launches the TurtleBot 4 Ignition simulation (Create 3 base +
RPLIDAR + OAK-D camera) + slam_toolbox via turtlebot4_navigation + Nav2 via
turtlebot4_navigation + apriltag_ros detector + RViz.

Exploration is launched separately (see exploration.launch.py).

Usage:
    # Full simulation stack
    ros2 launch team_b_bringup bringup.launch.py
    # Headless Ignition (no GUI)
    ros2 launch team_b_bringup bringup.launch.py headless:=true
    # Different world (depot, maze, warehouse...)
    ros2 launch team_b_bringup bringup.launch.py world:=maze
    # Only sim + SLAM (turn off Nav2 and AprilTag)
    ros2 launch team_b_bringup bringup.launch.py launch_nav2:=false launch_apriltag:=false

Prerequisites (apt — see README "Prérequis"):
    sudo apt install ros-humble-turtlebot4-simulator \
                     ros-humble-turtlebot4-navigation \
                     ros-humble-turtlebot4-msgs \
                     ros-humble-apriltag-ros \
                     ignition-fortress
"""
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_bringup = FindPackageShare('team_b_bringup')
    pkg_tb4_ignition = FindPackageShare('turtlebot4_ignition_bringup')
    pkg_tb4_navigation = FindPackageShare('turtlebot4_navigation')
    pkg_nav2_bringup = FindPackageShare('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    headless = LaunchConfiguration('headless')
    world = LaunchConfiguration('world')
    model = LaunchConfiguration('model')
    launch_rviz = LaunchConfiguration('launch_rviz')
    launch_slam = LaunchConfiguration('launch_slam')
    launch_nav2 = LaunchConfiguration('launch_nav2')
    launch_apriltag = LaunchConfiguration('launch_apriltag')
    algo = LaunchConfiguration('algo')
    run_id = LaunchConfiguration('run_id')

    slam_params = PathJoinSubstitution([pkg_bringup, 'config', 'slam_params.yaml'])
    apriltag_params = PathJoinSubstitution([pkg_bringup, 'config', 'apriltag_tags.yaml'])
    rviz_config = PathJoinSubstitution([pkg_bringup, 'rviz', 'sar.rviz'])
    nav2_params = PathJoinSubstitution([pkg_nav2_bringup, 'params', 'nav2_params.yaml'])

    # -- 1) TurtleBot 4 + Ignition Gazebo Fortress (Create 3 + RPLIDAR + OAK-D)
    tb4_ignition = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_tb4_ignition, 'launch', 'turtlebot4_ignition.launch.py'])
        ),
        launch_arguments={
            'world': world,
            'model': model,
            'rviz': 'false',           # we launch our own RViz below
            'slam': 'false',           # we launch our own SLAM below
            'nav2': 'false',           # we launch our own Nav2 below
            'localization': 'false',
            'headless': headless,
        }.items(),
    )

    # -- 2) SLAM Toolbox (async, loop-closure tuned) via turtlebot4_navigation wrapper
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_tb4_navigation, 'launch', 'slam.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params': slam_params,
        }.items(),
        condition=IfCondition(launch_slam),
    )

    # -- 3) Nav2 navigation servers (TB4 wrapper handles odom/scan remaps)
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_tb4_navigation, 'launch', 'nav2.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params,
        }.items(),
        condition=IfCondition(launch_nav2),
    )

    # -- 4) AprilTag detector — OAK-D RGB camera (rectified preview stream)
    apriltag = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag',
        output='screen',
        parameters=[apriltag_params, {'use_sim_time': use_sim_time}],
        remappings=[
            ('image_rect', '/oakd/rgb/preview/image_raw'),
            ('camera_info', '/oakd/rgb/preview/camera_info'),
        ],
        condition=IfCondition(launch_apriltag),
    )

    # -- 5) RViz with our project config (sar.rviz: Frame Rate clamped to 10 for perf)
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(launch_rviz),
    )

    return LaunchDescription([
        # ----- Args -----
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use the Ignition /clock for all nodes.'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='Run Ignition without GUI (CI/benchmark).'),
        DeclareLaunchArgument('world', default_value='warehouse',
                              description='Ignition world name (warehouse | depot | maze).'),
        DeclareLaunchArgument('model', default_value='standard',
                              description='TurtleBot 4 model variant (standard | lite).'),
        DeclareLaunchArgument('launch_rviz', default_value='true',
                              description='Launch RViz2.'),
        DeclareLaunchArgument('launch_slam', default_value='true',
                              description='Launch slam_toolbox (via turtlebot4_navigation).'),
        DeclareLaunchArgument('launch_nav2', default_value='true',
                              description='Launch Nav2 (via turtlebot4_navigation).'),
        DeclareLaunchArgument('launch_apriltag', default_value='true',
                              description='Launch apriltag_ros detector on OAK-D stream.'),
        DeclareLaunchArgument('algo', default_value='greedy',
                              choices=['greedy', 'info_gain'],
                              description='Exploration algo tag (consumed by exploration.launch.py).'),
        DeclareLaunchArgument('run_id', default_value='0',
                              description='Run identifier for benchmark logs.'),

        # ----- Actions -----
        LogInfo(msg=['[team_b_bringup] L15 TB4 bringup — world=', world,
                     ' model=', model,
                     ' algo=', algo,
                     ' headless=', headless]),
        tb4_ignition,
        slam,
        nav2,
        apriltag,
        rviz,
    ])
