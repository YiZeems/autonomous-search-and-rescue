# shellcheck shell=bash
# Platform profile: Windows 11 + WSL2, Ubuntu 22.04 x86_64.
#
# Sourced by scripts/sh/run_demo_tb4.sh. WSLg exposes the GPU to Linux through
# the Direct3D12 Mesa driver (/dev/dxg), so OpenGL is HARDWARE-accelerated —
# unlike the Parallels ARM64 case. Verified on this stack:
#   - Ogre2 (Ignition's default) works on the WSLg GPU, BUT the Mesa D3D12 driver
#     advertises only GL 4.1 Compatibility and Ogre2's GL3Plus path then aborts
#     in the sensor RenderThread — so we must force GL 4.5/GLSL 450 (below).
#   - On hybrid Intel+NVIDIA hosts, WSLg's D3D12 GL defaults to the weak iGPU;
#     we auto-select the discrete NVIDIA GPU (MESA_D3D12_DEFAULT_ADAPTER_NAME).
#   - With those two fixes the gpu_lidar is dense and the Gazebo GUI opens.
#
# Fallbacks: IA712_WSL_SOFTWARE_GL=1 forces the Ogre v1 + llvmpipe software path
# (no GPU at all); IA712_GPU_ADAPTER=Intel pins the iGPU instead of NVIDIA.

export IA712_PLATFORM_NAME="win-wsl2-x86_64"

# --- DDS middleware ---
# Fast-DDS' WSL discovery is flaky: the in-Ignition gz_ros2_control intermittently
# fails to discover the robot_state_publisher / controller_manager services, so the
# wheel controllers never load and /turtlebot4/odom stays silent. CycloneDDS
# discovers reliably on WSL. Install: sudo apt install ros-humble-rmw-cyclonedds-cpp.
# (Set before any ROS process so the ign gazebo plugin inherits it too.)
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

# Pin CycloneDDS to the loopback interface. Everything runs on this single WSL
# host, and WSL's external NIC has flaky multicast, which makes node discovery
# intermittent (e.g. lifecycle_manager never finds planner_server -> Nav2 never
# activates). Loopback is deterministic and reliable for all-local discovery.
export CYCLONEDDS_URI="${CYCLONEDDS_URI:-<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"lo\" multicast=\"true\"/></Interfaces></General></Domain></CycloneDDS>}"

# --- Render engine ---
# Some WSLg machines expose OpenGL only through the Mesa *D3D12* driver
# (no /dev/dri, just /dev/dxg + libd3d12.so). That driver cannot run Ignition's
# default Ogre2 GL3Plus path: `ign gazebo` aborts in the Sensors RenderThread
# (Ogre2Material::SetTextureMapImpl, see docs/ERRORS_AND_FIXES.md #10).
# On such a machine, set IA712_WSL_SOFTWARE_GL=1 (or put the same exports in
# config/local_env.sh) to fall back to the Ogre v1 + llvmpipe software recipe:
#   IA712_WSL_SOFTWARE_GL=1 ./scripts/run.sh demo-tb4
if [ "${IA712_WSL_SOFTWARE_GL:-0}" = "1" ]; then
  # Software-GL fallback: Ogre v1 + llvmpipe. Slower and the gpu_lidar is sparse,
  # but it runs with no GPU at all. Only needed if the WSLg GL override path below
  # does not work on your machine.
  export IA712_RENDER_ENGINE="${IA712_RENDER_ENGINE:-ogre}"
  export LIBGL_ALWAYS_SOFTWARE=1
  export MESA_GL_VERSION_OVERRIDE="${MESA_GL_VERSION_OVERRIDE:-3.3}"
  export MESA_GLSL_VERSION_OVERRIDE="${MESA_GLSL_VERSION_OVERRIDE:-330}"
else
  # Default: Ogre2 on the WSLg HARDWARE GPU (e.g. D3D12 -> Intel/AMD/NVIDIA).
  # The Mesa D3D12 driver advertises GL 4.1 *Compatibility*, but Ogre2's GL3Plus
  # render system needs a higher version exposed or it aborts in the sensor
  # RenderThread (Ogre2Material::SetTextureMapImpl, ERRORS_AND_FIXES #10).
  # Forcing GL 4.5 / GLSL 450 fixes the crash and gives a dense gpu_lidar.
  export IA712_RENDER_ENGINE="${IA712_RENDER_ENGINE:-ogre2}"
  export MESA_GL_VERSION_OVERRIDE="${MESA_GL_VERSION_OVERRIDE:-4.5}"
  export MESA_GLSL_VERSION_OVERRIDE="${MESA_GLSL_VERSION_OVERRIDE:-450}"
  unset LIBGL_ALWAYS_SOFTWARE

  # Hybrid GPU hosts (Intel iGPU + discrete NVIDIA/AMD) make WSLg's D3D12 GL pick
  # the weaker iGPU by default. Prefer the discrete NVIDIA GPU when nvidia-smi
  # sees one — much faster rendering + a denser gpu_lidar. Override or disable
  # with IA712_GPU_ADAPTER (e.g. IA712_GPU_ADAPTER=Intel, or ="" to let WSL pick).
  if [ -z "${MESA_D3D12_DEFAULT_ADAPTER_NAME:-}" ]; then
    if [ -n "${IA712_GPU_ADAPTER+x}" ]; then
      [ -n "${IA712_GPU_ADAPTER}" ] && export MESA_D3D12_DEFAULT_ADAPTER_NAME="${IA712_GPU_ADAPTER}"
    elif command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi >/dev/null 2>&1; then
      export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
    fi
  fi
fi
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"
export DISPLAY="${DISPLAY:-:0}"

# --- DDS transport ---
# Keep the default Fast-DDS SHM transport on WSL2: SHM discovery is what lets the
# in-Ignition gz_ros2_control find the spawn's robot_state_publisher service, so
# the controllers load and /turtlebot4/odom publishes. (UDP-only DDS breaks that
# discovery here because WSL multicast is flaky.) The real SHM problem was *stale*
# segments piling up across runs (ERRORS_AND_FIXES #21); IA712_CLEAN_SHM below now
# purges them thoroughly before each run, so SHM stays healthy.
# Set IA712_USE_UDP_DDS=1 only if your box genuinely needs UDP-only.
export IA712_USE_UDP_DDS="${IA712_USE_UDP_DDS:-0}"
export IA712_CLEAN_SHM=1

# --- GUI: on by default (normal Ogre2 window through WSLg) ---
export IA712_TB4_GUI="${IA712_TB4_GUI:-1}"
