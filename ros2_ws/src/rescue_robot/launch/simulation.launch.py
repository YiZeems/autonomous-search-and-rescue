import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


SUPPORTED_UPSTREAM_WORLDS = {
    "house": "turtlebot3_house.launch.py",
    "world": "turtlebot3_world.launch.py",
    "base": "turtlebot3_world.launch.py",
    "empty": "empty_world.launch.py",
}


def include_turtlebot3_world(context, *args, **kwargs):
    """Include a TurtleBot3 Gazebo world or launch a custom .world file directly.

    Priority:
    1. If world_file arg is set to an existing .world path → launch gzserver directly
       with that file (supports any custom world, including project worlds/).
    2. Otherwise use the world arg to pick an upstream TurtleBot3 launch file.
       Falls back to turtlebot3_world.launch.py if the requested file is missing.
    """

    world_file = LaunchConfiguration("world_file").perform(context).strip()
    if world_file:
        if not os.path.isabs(world_file):
            # Project custom worlds live in rescue_world, not rescue_robot.
            # rescue_robot/setup.py does not install a worlds/ directory.
            try:
                pkg_share = get_package_share_directory("rescue_world")
            except Exception:
                pkg_share = get_package_share_directory("rescue_robot")
            world_file = os.path.join(pkg_share, "worlds", world_file)
        if os.path.exists(world_file):
            return [
                LogInfo(msg=f"Loading custom world file: {world_file}"),
                ExecuteProcess(
                    cmd=["gzserver", "--verbose", world_file,
                         "-s", "libgazebo_ros_init.so",
                         "-s", "libgazebo_ros_factory.so"],
                    output="screen",
                ),
                ExecuteProcess(
                    cmd=["gzclient"],
                    output="screen",
                ),
            ]
        else:
            return [LogInfo(
                msg=f"[WARN] world_file not found: {world_file}. Falling back to world arg."
            )]

    world_key = LaunchConfiguration("world").perform(context).strip().lower()
    launch_file = SUPPORTED_UPSTREAM_WORLDS.get(world_key, world_key)

    if not launch_file.endswith(".launch.py"):
        launch_file = f"{launch_file}.launch.py"

    turtlebot3_gazebo_share = get_package_share_directory("turtlebot3_gazebo")
    launch_dir = os.path.join(turtlebot3_gazebo_share, "launch")
    launch_path = os.path.join(launch_dir, launch_file)

    fallback_file = "turtlebot3_world.launch.py"
    fallback_path = os.path.join(launch_dir, fallback_file)

    actions = []
    if not os.path.exists(launch_path):
        actions.append(
            LogInfo(
                msg=(
                    f"Requested TurtleBot3 Gazebo world launch '{launch_file}' "
                    f"was not found. Falling back to '{fallback_file}'."
                )
            )
        )
        launch_path = fallback_path
        launch_file = fallback_file

    actions.append(LogInfo(msg=f"Using TurtleBot3 Gazebo launch file: {launch_file}"))
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_path),
        )
    )
    return actions


def generate_launch_description():
    """Launch a TurtleBot3 Gazebo simulation.

    Default world is 'house' (corridors + rooms, close to rescue scenario).
    To load any custom .world file:
        ros2 launch rescue_robot simulation.launch.py world_file:=disaster_world.world
    or with an absolute path:
        ros2 launch rescue_robot simulation.launch.py world_file:=/path/to/my.world
    """

    model_arg = DeclareLaunchArgument(
        "model",
        default_value="waffle_pi",
        description="TurtleBot3 model: burger, waffle, or waffle_pi.",
    )

    world_arg = DeclareLaunchArgument(
        "world",
        default_value="house",
        description=(
            "TurtleBot3 upstream world key: house, world/base, empty. "
            "Ignored if world_file is set."
        ),
    )

    world_file_arg = DeclareLaunchArgument(
        "world_file",
        default_value="",
        description=(
            "Path to a custom .world file. Relative paths are resolved from "
            "the rescue_world package share (worlds/). "
            "Example: world_file:=disaster_world.world"
        ),
    )

    return LaunchDescription(
        [
            model_arg,
            world_arg,
            world_file_arg,
            SetEnvironmentVariable(
                name="TURTLEBOT3_MODEL",
                value=LaunchConfiguration("model"),
            ),
            LogInfo(
                msg=[
                    "Starting TurtleBot3 Gazebo simulation — model=",
                    LaunchConfiguration("model"),
                    ", world=",
                    LaunchConfiguration("world"),
                    ", world_file=",
                    LaunchConfiguration("world_file"),
                ]
            ),
            OpaqueFunction(function=include_turtlebot3_world),
        ]
    )
