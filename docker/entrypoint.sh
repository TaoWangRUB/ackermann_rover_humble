#!/bin/bash
set -e

# Base ROS 2 distro (default to jazzy if not set)
: "${ROS_DISTRO:=jazzy}"

FASTDDS_PROFILE="/workspace/config/fastdds_eth_only.xml"
FASTDDS_SHELL_HOOK="${HOME}/.fastdds_eth_env.sh"

cat > "${FASTDDS_SHELL_HOOK}" <<'EOF'
#!/bin/bash
_fastdds_profile="/workspace/config/fastdds_eth_only.xml"

if [[ -f "${_fastdds_profile}" ]]; then
  if [[ -z "${ROS_ETH_INTERFACE_IP:-}" ]]; then
    _detected_eth_ip="$(
      hostname -I 2>/dev/null \
      | tr ' ' '\n' \
      | awk '/^10\.42\./ { print; exit }'
    )"
    if [[ -n "${_detected_eth_ip}" ]]; then
      export ROS_ETH_INTERFACE_IP="${_detected_eth_ip}"
    fi
  fi

  if [[ -n "${ROS_ETH_INTERFACE_IP:-}" ]]; then
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    export FASTRTPS_DEFAULT_PROFILES_FILE="${_fastdds_profile}"
    export FASTDDS_DEFAULT_PROFILES_FILE="${_fastdds_profile}"
  fi
fi

unset _fastdds_profile _detected_eth_ip
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

# Update apt index and install ROS package dependencies via rosdep
sudo apt-get update -qq
source "/opt/ros/${ROS_DISTRO}/setup.bash"
rosdep install --from-paths src --ignore-src -r -y || \
  echo "WARNING: rosdep install encountered errors; continuing startup."

# Source overlay workspace if it has been built
if [ -f /workspace/install/setup.bash ]; then
  source /workspace/install/setup.bash
fi

# Hand off to the requested command (bash by default)
exec "$@"
