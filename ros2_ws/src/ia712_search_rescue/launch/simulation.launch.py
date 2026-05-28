import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
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
    """Include a TurtleBot3 Gazebo launch file based on the selected world.

    The project default is the TurtleBot3 house world because it is closer to a
    search-and-rescue environment than the basic demo world: it has corridors,
    rooms and occlusions. If the selected launch file is missing in an upstream
    TurtleBot3 overlay, we fall back to turtlebot3_world.launch.py.
    """

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
    """Launch a real TurtleBot3 Gazebo simulation.

    This launch file is a real simulation smoke test. It starts upstream
    TurtleBot3 Gazebo with a selectable world. The default world is 'house' so
    the team can test in a map closer to the rescue scenario than the basic
    TurtleBot3 demo world.
    """

    model_arg = DeclareLaunchArgument(
        "model",
        default_value="waffle_pi",
        description="TurtleBot3 model to use: burger, waffle, or waffle_pi.",
    )

    world_arg = DeclareLaunchArgument(
        "world",
        default_value="house",
        description=(
            "TurtleBot3 Gazebo world to launch: house, world/base, empty, "
            "or an upstream launch filename without extension."
        ),
    )

    return LaunchDescription(
        [
            model_arg,
            world_arg,
            SetEnvironmentVariable(
                name="TURTLEBOT3_MODEL",
                value=LaunchConfiguration("model"),
            ),
            LogInfo(
                msg=[
                    "Starting real TurtleBot3 Gazebo simulation with model=",
                    LaunchConfiguration("model"),
                    ", world=",
                    LaunchConfiguration("world"),
                ]
            ),
            OpaqueFunction(function=include_turtlebot3_world),
        ]
    )
