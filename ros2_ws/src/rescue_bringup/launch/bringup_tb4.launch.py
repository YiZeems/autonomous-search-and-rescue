"""TurtleBot 4 + Ignition Gazebo Fortress — SIMPLE bringup (alternative).

Brought in from autonomous-search-and-rescue/ during the bl/ merge.
Runtime-validated 2026-06-09 on a clean WSL2 RTX (60 nodes alive, topics
publishing, single Ignition instance).

WHEN TO USE THIS vs ./scripts/run.sh demo-tb4:

- This launch is **simple**: it brings up the stack (Ignition + Create 3 +
  RPLIDAR + OAK-D + SLAM + Nav2 + AprilTag + RViz) and lets you drive it
  manually (e.g. RViz 2D Pose Goal) or hook up your own exploration.
- The shell script `./scripts/run.sh demo-tb4` (501 lines, see
  scripts/sh/run_demo_tb4.sh) is the **validated end-to-end** demo: it adds
  CycloneDDS profile, TF namespace bridges (tf_relay), cmd_vel relay,
  scan_throttle, kill-sim CPU cleanup, and chains the frontier_explorer_node
  to reach 91.4% coverage autonomously. Use it for actual demos.

Use:
    # Simple full TB4 stack
    ros2 launch rescue_bringup bringup_tb4.launch.py
    # Headless Ignition
    ros2 launch rescue_bringup bringup_tb4.launch.py headless:=true
    # Different world (warehouse default, also depot, maze)
    ros2 launch rescue_bringup bringup_tb4.launch.py world:=maze
    # Toggle bricks for isolation tests
    ros2 launch rescue_bringup bringup_tb4.launch.py launch_nav2:=false launch_apriltag:=false

Prerequisites (apt):
    sudo apt install ros-humble-turtlebot4-simulator \\
                     ros-humble-turtlebot4-ignition-bringup \\
                     ros-humble-turtlebot4-navigation \\
                     ros-humble-turtlebot4-msgs \\
                     ros-humble-irobot-create-msgs \\
                     ros-humble-ros-gz-bridge \\
                     ros-humble-apriltag-ros \\
                     ignition-fortress
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_bringup = FindPackageShare('rescue_bringup')
    pkg_rescue_robot = FindPackageShare('rescue_robot')
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

    # SLAM params: TB4-specific file with fixed RPLIDAR A1M8 range (12 m, not 3.5).
    slam_params = PathJoinSubstitution(
        [pkg_rescue_robot, 'config', 'slam_params_tb4.yaml']
    )
    apriltag_params = PathJoinSubstitution(
        [pkg_bringup, 'config', 'apriltag_tags.yaml']
    )
    nav2_default_params = PathJoinSubstitution(
        [pkg_nav2_bringup, 'params', 'nav2_params.yaml']
    )
    rviz_config = PathJoinSubstitution(
        [pkg_rescue_robot, 'rviz', 'project_view.rviz']
    )

    # 1) TurtleBot 4 + Ignition Gazebo Fortress
    tb4_ignition = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_tb4_ignition, 'launch', 'turtlebot4_ignition.launch.py'])
        ),
        launch_arguments={
            'world': world,
            'model': model,
            'rviz': 'false',
            'slam': 'false',
            'nav2': 'false',
            'localization': 'false',
            'headless': headless,
        }.items(),
    )

    # 2) SLAM Toolbox via turtlebot4_navigation wrapper, with explicit params (no race).
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

    # 3) Nav2 via turtlebot4_navigation wrapper. params_file passed explicitly to
    #    sidestep the [Errno 2] race (see docs/ERRORS_AND_FIXES.md #27).
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_tb4_navigation, 'launch', 'nav2.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_default_params,
        }.items(),
        condition=IfCondition(launch_nav2),
    )

    # 4) AprilTag detector — OAK-D RGB camera (rectified preview stream).
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

    # 5) RViz with project_view config (Frame Rate clamped to 10 for perf).
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(launch_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use the Ignition /clock for all nodes.'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='Run Ignition without GUI (CI/benchmark).'),
        DeclareLaunchArgument('world', default_value='warehouse',
                              description='Ignition world name (warehouse | depot | maze).'),
        DeclareLaunchArgument('model', default_value='standard',
                              description='TurtleBot 4 variant (standard ships the OAK-D).'),
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
        LogInfo(msg=['[rescue_bringup] simple TB4 bringup — world=', world,
                     ' model=', model, ' algo=', algo, ' headless=', headless,
                     ' (for the validated end-to-end demo, use ./scripts/run.sh demo-tb4 instead)']),
        tb4_ignition,
        slam,
        nav2,
        apriltag,
        rviz,
    ])
