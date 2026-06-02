"""Single entry-point launch file for the IA712 Project B Search & Rescue stack.

L15: launches Gazebo (turtlebot3_house by default) + TurtleBot3 Waffle Pi +
slam_toolbox (async mapping, loop closure tuned) + Nav2 navigation servers +
apriltag_ros detector + RViz.

Exploration is launched separately (see exploration.launch.py) so each brick
can be validated in isolation per the L15 deliverable.

Usage:
    # Full simulation stack
    ros2 launch rescue_bringup bringup.launch.py
    # Headless (no Gazebo GUI, RViz still on)
    ros2 launch rescue_bringup bringup.launch.py headless:=true
    # Only Gazebo + SLAM (turn off nav2 and apriltag for SLAM-only testing)
    ros2 launch rescue_bringup bringup.launch.py launch_nav2:=false launch_apriltag:=false
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# IMPORTANT: TURTLEBOT3_MODEL is read at module-import time by the bundled
# turtlebot3_gazebo launch files (robot_state_publisher.launch.py and
# spawn_turtlebot3.launch.py both do `os.environ['TURTLEBOT3_MODEL']` inside
# their generate_launch_description). A SetEnvironmentVariable action would
# fire too late — so we set it here, before any IncludeLaunchDescription
# triggers their generate().
os.environ.setdefault('TURTLEBOT3_MODEL', 'waffle_pi')


def generate_launch_description() -> LaunchDescription:
    pkg_bringup = FindPackageShare('rescue_bringup')
    pkg_rescue_robot = FindPackageShare('rescue_robot')
    pkg_tb3_gazebo = FindPackageShare('turtlebot3_gazebo')
    pkg_slam_toolbox = FindPackageShare('slam_toolbox')
    pkg_nav2_bringup = FindPackageShare('nav2_bringup')
    pkg_gazebo_ros = FindPackageShare('gazebo_ros')

    # Launch configurations (kept as substitutions so they propagate)
    use_sim_time = LaunchConfiguration('use_sim_time')
    headless = LaunchConfiguration('headless')
    world = LaunchConfiguration('world')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    launch_rviz = LaunchConfiguration('launch_rviz')
    launch_slam = LaunchConfiguration('launch_slam')
    launch_nav2 = LaunchConfiguration('launch_nav2')
    launch_apriltag = LaunchConfiguration('launch_apriltag')
    algo = LaunchConfiguration('algo')
    run_id = LaunchConfiguration('run_id')

    slam_params = PathJoinSubstitution([pkg_bringup, 'config', 'slam_params.yaml'])
    apriltag_params = PathJoinSubstitution([pkg_bringup, 'config', 'apriltag_tags.yaml'])

    # Project RViz config: map, lidar scan, camera feed, costmap, robot model, victim markers
    rviz_config = PathJoinSubstitution(
        [pkg_rescue_robot, 'rviz', 'project_view.rviz']
    )

    # -- Gazebo: split gzserver / gzclient so headless works
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_gazebo_ros, 'launch', 'gzserver.launch.py'])
        ),
        launch_arguments={'world': world}.items(),
    )
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_gazebo_ros, 'launch', 'gzclient.launch.py'])
        ),
        condition=UnlessCondition(headless),
    )

    # -- TurtleBot3: robot_state_publisher + spawn
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_tb3_gazebo, 'launch', 'robot_state_publisher.launch.py'])
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )
    spawn_tb3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_tb3_gazebo, 'launch', 'spawn_turtlebot3.launch.py'])
        ),
        launch_arguments={'x_pose': x_pose, 'y_pose': y_pose}.items(),
    )

    # -- SLAM Toolbox (async, with our loop-closure-tuned params)
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_slam_toolbox, 'launch', 'online_async_launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_params,
        }.items(),
        condition=IfCondition(launch_slam),
    )

    # -- Nav2 navigation servers (no map_server / amcl — slam_toolbox provides /map)
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_nav2_bringup, 'launch', 'navigation_launch.py'])
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(launch_nav2),
    )

    # -- AprilTag detector (camera_36h11)
    apriltag = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag',
        output='screen',
        parameters=[apriltag_params, {'use_sim_time': use_sim_time}],
        remappings=[
            ('image_rect', '/camera/image_raw'),
            ('camera_info', '/camera/camera_info'),
        ],
        condition=IfCondition(launch_apriltag),
    )

    # -- RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(launch_rviz),
    )

    # Default world: turtlebot3_house — resolved at launch time so the file
    # parses cleanly even without the TB3 overlay in the current environment.
    try:
        default_world = os.path.join(
            get_package_share_directory('turtlebot3_gazebo'),
            'worlds', 'turtlebot3_house.world'
        )
    except Exception:
        default_world = ''

    return LaunchDescription([
        # ----- Args -----
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use Gazebo /clock for all nodes.'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='Run Gazebo without GUI (CI/benchmark).'),
        DeclareLaunchArgument('world', default_value=default_world,
                              description='Path to the .world file Gazebo loads.'),
        DeclareLaunchArgument('x_pose', default_value='-2.0',
                              description='Initial x of TurtleBot3 spawn.'),
        DeclareLaunchArgument('y_pose', default_value='-0.5',
                              description='Initial y of TurtleBot3 spawn.'),
        DeclareLaunchArgument('launch_rviz', default_value='true',
                              description='Launch RViz2.'),
        DeclareLaunchArgument('launch_slam', default_value='true',
                              description='Launch slam_toolbox async mapping.'),
        DeclareLaunchArgument('launch_nav2', default_value='true',
                              description='Launch Nav2 navigation servers.'),
        DeclareLaunchArgument('launch_apriltag', default_value='true',
                              description='Launch apriltag_ros detector.'),
        DeclareLaunchArgument('algo', default_value='greedy',
                              choices=['greedy', 'info_gain'],
                              description='Exploration algo (consumed by exploration.launch.py).'),
        DeclareLaunchArgument('run_id', default_value='0',
                              description='Run identifier for benchmark logs.'),

        # ----- Actions -----
        LogInfo(msg=[
            '[rescue_bringup] L15 bringup — algo=', algo,
            ' run_id=', run_id, ' headless=', headless,
        ]),
        gzserver,
        gzclient,
        robot_state_publisher,
        spawn_tb3,
        slam,
        nav2,
        apriltag,
        rviz,
    ])
