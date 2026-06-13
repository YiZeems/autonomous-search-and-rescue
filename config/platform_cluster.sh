# shellcheck shell=bash
# Platform profile: HPC compute node inside an Apptainer/Singularity container
# (mesogip ENSTA-l40s/h100). Selected with IA712_PLATFORM=cluster.
#
# Why each setting (cf. cloud_technique_2xcloud_robotique_ros.md + docs/ERRORS_AND_FIXES.md):
#   - NO --nv on the container → no host NVIDIA GL (GLIBC 2.38 vs 2.35 crash, piège #15).
#     The sim renders with Mesa **software GL** (llvmpipe), already in the image.
#   - Ogre v1: Ogre2's gpu_lidar depth pass fails under software GL → RPLIDAR
#     returns range_min on every ray (#10). Ogre v1 renders real ranges.
#   - Fully headless: no GUI, no RViz, no X display on the compute node.
#   - Fast-DDS UDP-only + SHM cleanup: avoid stale /dev/shm lockups (#21).
#   - llvmpipe uses the 8 allocated CPUs for software rendering.

export IA712_PLATFORM_NAME="cluster-apptainer-l40s"

# --- Render engine ---
# Ogre2 (NOT Ogre v1): the warehouse/TB4 worlds use PBR materials that Ogre v1
# can't render → Ogre v1 core-dumps. llvmpipe exposes GL 4.5, which Ogre2 needs,
# so Ogre2 runs in pure software here (slow but correct). (Differs from the mac
# Parallels profile where the vGPU forced Ogre v1.)
export IA712_RENDER_ENGINE="ogre2"

# --- Software GL (no --nv, Mesa llvmpipe — let it expose its native GL 4.5) ---
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
export GALLIUM_DRIVER="${GALLIUM_DRIVER:-llvmpipe}"
export LP_NUM_THREADS="${LP_NUM_THREADS:-8}"
# NB: do NOT force MESA_GL_VERSION_OVERRIDE (Ogre2 needs >= 4.5; llvmpipe gives it).

# --- Headless: no GUI, no display on the compute node ---
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-offscreen}"
export IA712_TB4_GUI=0
export IA712_RVIZ=0
unset DISPLAY 2>/dev/null || true

# --- DDS transport (avoid stale SHM lockups; Fast-DDS is the image default) ---
export IA712_USE_UDP_DDS=1
export IA712_CLEAN_SHM=1
