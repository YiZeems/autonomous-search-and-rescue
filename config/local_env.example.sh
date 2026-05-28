#!/usr/bin/env bash
# Optional local machine configuration. Do not commit config/local_env.sh.
# Copy this file to config/local_env.sh and adapt it if your machine needs a
# TurtleBot3 Gazebo source overlay, especially on ARM64.

# Option 1: point directly to a setup file.
# export IA712_TURTLEBOT3_OVERLAY_SETUP="/absolute/path/to/ros2_ws/install/setup.bash"

# Option 2: point to an install directory containing setup.bash or setup.zsh.
# export IA712_TURTLEBOT3_OVERLAY_INSTALL="/absolute/path/to/ros2_ws/install"


# Optional TurtleBot4 simulator overlay.
# Use this only if TurtleBot4 packages are not installed through apt.
# export IA712_TB4_OVERLAY_INSTALL="/absolute/path/to/turtlebot4_ws/install"
# export IA712_TB4_OVERLAY_SETUP="/absolute/path/to/turtlebot4_ws/install/setup.bash"

# Optional TurtleBot4 launch overrides.
# Defaults target the common ROS 2 Humble Ignition stack:
#   package: turtlebot4_ignition_bringup
#   launch:  turtlebot4_ignition.launch.py
# export IA712_TB4_BRINGUP_PACKAGE="turtlebot4_ignition_bringup"
# export IA712_TB4_LAUNCH_FILE="turtlebot4_ignition.launch.py"
# export IA712_TB4_MODEL="standard"
# export IA712_TB4_WORLD="depot"
