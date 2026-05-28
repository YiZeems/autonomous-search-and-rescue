from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Project bringup entrypoint.

    Current V1 behavior: launch the real TurtleBot3 Gazebo simulation so the
    team can validate the real robot/sensor interface. The mock system remains
    available separately with `./scripts/run.sh mock`.

    Future V2/V3 behavior: progressively include navigation, exploration,
    detection, results and Behavior Tree launch files.
    """

    simulation_launch = PathJoinSubstitution(
        [
            FindPackageShare("ia712_search_rescue"),
            "launch",
            "simulation.launch.py",
        ]
    )

    return LaunchDescription(
        [
            LogInfo(msg="IA712 bringup V1: launching real TurtleBot3 simulation."),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(simulation_launch)),
        ]
    )
