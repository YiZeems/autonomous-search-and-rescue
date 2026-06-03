# shellcheck shell=bash
# Detect the host platform and source the matching profile under config/.
#
#   - Windows 11 + WSL2 (x86_64)  -> config/platform_win.sh   (hardware GPU via WSLg)
#   - macOS + Parallels (aarch64) -> config/platform_mac.sh   (software GL, Ogre v1)
#
# Override the auto-detection with:  IA712_PLATFORM=mac   or   IA712_PLATFORM=win
#
# Exposes (consumed by run_demo_tb4.sh): IA712_RENDER_ENGINE, IA712_USE_UDP_DDS,
# IA712_CLEAN_SHM, IA712_TB4_GUI, IA712_PLATFORM_NAME, and the GL/Qt env.

detect_platform() {
  if [ -n "${IA712_PLATFORM:-}" ]; then
    echo "${IA712_PLATFORM}"
    return 0
  fi
  # WSL exposes "microsoft" / "WSL" in the kernel string
  if grep -qiE "microsoft|wsl" /proc/sys/kernel/osrelease 2>/dev/null \
     || grep -qiE "microsoft|wsl" /proc/version 2>/dev/null; then
    echo "win"
    return 0
  fi
  # ARM64 with no WSL marker -> assume Parallels on Apple Silicon
  case "$(uname -m)" in
    aarch64|arm64) echo "mac" ;;
    *)             echo "win" ;;   # x86_64 bare-metal/other: treat like the hardware-GPU path
  esac
}

source_platform_profile() {
  local repo_root="$1"
  local platform
  platform="$(detect_platform)"
  local profile="${repo_root}/config/platform_${platform}.sh"
  if [ -f "${profile}" ]; then
    # shellcheck disable=SC1090
    source "${profile}"
    echo "[platform] detected '${platform}' -> ${IA712_PLATFORM_NAME:-?}  (render-engine: ${IA712_RENDER_ENGINE:-default})"
  else
    echo "[platform] WARNING: profile not found: ${profile} (continuing with defaults)" >&2
  fi
}
