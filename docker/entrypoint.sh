#!/bin/bash
set -e

# Base ROS 2 distro (default to jazzy if not set)
: "${ROS_DISTRO:=jazzy}"
: "${ROS_DDS_MODE:=eth+shm}"

FASTDDS_SHELL_HOOK="${HOME}/.fastdds_eth_env.sh"

cat > "${FASTDDS_SHELL_HOOK}" <<'EOF'
#!/bin/bash
_eth_only_profile="/workspace/config/fastdds_eth_only.xml"
_eth_shm_profile="/workspace/config/fastdds_eth_shm.xml"
_fastdds_profile=""
_ros_dds_mode="${ROS_DDS_MODE:-eth+shm}"

_detect_eth_ip() {
  hostname -I 2>/dev/null \
    | tr ' ' '\n' \
    | awk '/^10\.42\./ { print; exit }'
}

# Reset profile-related variables each time the hook runs so switching modes
# in a fresh shell reliably changes the DDS transport configuration.
unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE

if [[ -z "${RMW_IMPLEMENTATION:-}" ]]; then
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
fi

case "${_ros_dds_mode}" in
  local)
    ;;
  eth+shm|eth_shm|eth-shm)
    _fastdds_profile="${_eth_shm_profile}"
    ;;
  eth-only|eth_only|eth)
    _fastdds_profile="${_eth_only_profile}"
    ;;
  *)
    echo "Unknown ROS_DDS_MODE='${_ros_dds_mode}'. Supported values: eth+shm, eth-only, local. Falling back to local." >&2
    _ros_dds_mode="local"
    ;;
esac

if [[ -n "${_fastdds_profile}" ]]; then
  if [[ -z "${ROS_ETH_INTERFACE_IP:-}" ]]; then
    _detected_eth_ip="$(_detect_eth_ip)"
    if [[ -n "${_detected_eth_ip}" ]]; then
      export ROS_ETH_INTERFACE_IP="${_detected_eth_ip}"
    fi
  fi

  if [[ -n "${ROS_ETH_INTERFACE_IP:-}" ]]; then
    export FASTRTPS_DEFAULT_PROFILES_FILE="${_fastdds_profile}"
    export FASTDDS_DEFAULT_PROFILES_FILE="${_fastdds_profile}"
  else
    echo "ROS_DDS_MODE=${_ros_dds_mode}, but no 10.42.x.x ethernet IP was detected; skipping Fast DDS profile." >&2
  fi
fi

export ROS_DDS_MODE="${_ros_dds_mode}"

unset _eth_only_profile _eth_shm_profile _fastdds_profile _ros_dds_mode _detect_eth_ip _detected_eth_ip
EOF
chmod +x "${FASTDDS_SHELL_HOOK}"

ensure_shell_hook() {
  local target_file="$1"
  local hook_line='[ -f "$HOME/.fastdds_eth_env.sh" ] && source "$HOME/.fastdds_eth_env.sh"'

  touch "${target_file}"
  if ! grep -Fqx "${hook_line}" "${target_file}"; then
    printf '\n%s\n' "${hook_line}" >> "${target_file}"
  fi
}

ensure_shell_hook "${HOME}/.bashrc"
ensure_shell_hook "${HOME}/.profile"

# Load the Fast DDS shell hook for the initial container command too.
source "${FASTDDS_SHELL_HOOK}"

if [[ -n "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" ]]; then
  echo "ROS DDS mode: ${ROS_DDS_MODE} (profile: ${FASTRTPS_DEFAULT_PROFILES_FILE})"
else
  echo "ROS DDS mode: ${ROS_DDS_MODE} (Fast DDS builtin transports)"
fi

# Aggressive TCP retransmit budget so wedged MQTT/TCP sockets fail fast (~12-25 s)
# instead of sitting in the default ~15 min limbo when WiFi has loss bursts.
# Paho C++ 1.2 doesn't expose per-socket TCP_USER_TIMEOUT — once we upgrade to
# 1.4+ we can move this to a setsockopt() in telemetry_publisher and drop the
# kernel-wide setting. Until then, sysctl is the cheapest correct-enough fix.
sudo sysctl -w net.ipv4.tcp_retries2=5 >/dev/null 2>&1 || \
  echo "WARNING: failed to set net.ipv4.tcp_retries2; continuing."

# Update apt index and install ROS package dependencies via rosdep.
# apt-get update can fail with "Release file is not valid yet" when the
# upstream mirror serves future-dated Release files (intermittent on
# ports.ubuntu.com); don't kill the container for that — rosdep can still
# resolve against the existing apt cache.
sudo apt-get update -qq || \
  echo "WARNING: apt-get update encountered errors; continuing startup."
source "/opt/ros/${ROS_DISTRO}/setup.bash"
rosdep install --from-paths src --ignore-src -r -y || \
  echo "WARNING: rosdep install encountered errors; continuing startup."

# Source overlay workspace if it has been built
if [ -f /workspace/install/setup.bash ]; then
  source /workspace/install/setup.bash
fi

# Hand off to the requested command (bash by default)
exec "$@"
