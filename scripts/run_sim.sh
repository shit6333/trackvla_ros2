#!/usr/bin/env bash
# Start the headless simulator and the ROS bridge.
# Intended to run inside the gazebo container, from the workspace root.
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/.."

set +u
# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u

exec ros2 launch sim/launch/sim.launch.py "$@"
