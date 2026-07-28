#!/bin/bash
set -e

source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "${SPOT_WS}/install/setup.bash"

exec "$@"
