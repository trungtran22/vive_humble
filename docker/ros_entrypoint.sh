#!/usr/bin/env bash
set -e

# Source the ROS 2 underlay and the vive_ros overlay.
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
if [[ -f "${ROS_WS:-/ros2_ws}/install/setup.bash" ]]; then
    # shellcheck source=/dev/null
    source "${ROS_WS:-/ros2_ws}/install/setup.bash"
fi

exec "$@"
