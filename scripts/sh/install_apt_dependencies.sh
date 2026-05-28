#!/usr/bin/env bash
set -eo pipefail

if [ ! -f /opt/ros/humble/setup.bash ]; then
  echo "[ERROR] /opt/ros/humble/setup.bash not found." >&2
  echo "Install ROS 2 Humble first, then rerun this script." >&2
  exit 1
fi

sudo apt update
sudo apt install -y \
  git \
  curl \
  tree \
  python3-pip \
  python3-rosdep \
  python3-vcstool \
  python3-colcon-common-extensions \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-turtlebot3 \
  ros-humble-tf2-tools \
  ros-humble-rviz2 \
  ros-humble-robot-state-publisher \
  ros-humble-joint-state-publisher \
  ros-humble-cv-bridge \
  ros-humble-image-transport

# TurtleBot3 Gazebo packages are usually available on AMD64/WSL2, but may be
# missing from apt on ARM64. Install them only when apt can locate them.
optional_packages=(
  ros-humble-turtlebot3-gazebo
  ros-humble-turtlebot3-simulations
  ros-humble-turtlebot3-bringup
  ros-humble-turtlebot3-description
  ros-humble-teleop-twist-keyboard
  ros-humble-turtlebot4-simulator
  ros-humble-turtlebot4-desktop
)

available_optional=()
for pkg in "${optional_packages[@]}"; do
  if apt-cache show "$pkg" >/dev/null 2>&1; then
    available_optional+=("$pkg")
  else
    echo "[WARN] Optional apt package not available on this architecture/repo: $pkg"
  fi
done

if [ "${#available_optional[@]}" -gt 0 ]; then
  sudo apt install -y "${available_optional[@]}"
fi

sudo rosdep init 2>/dev/null || true
rosdep update

echo "[OK] Project apt dependencies installed."
echo "[INFO] If turtlebot3_gazebo is missing on ARM64, use a source overlay."
echo "[INFO] Check TurtleBot3 with: ./scripts/run.sh check-tb3"
echo "[INFO] Check TurtleBot4 with: ./scripts/run.sh check-tb4"
