#!/usr/bin/env bash
# Sources ROS and points Gazebo at the simulation assets in this repository.
set -e

source "/opt/ros/${ROS_DISTRO}/setup.bash"

SIM_ROOT=/workspace/trackvla_ros2/sim
if [ -d "${SIM_ROOT}/models" ]; then
    export GZ_SIM_RESOURCE_PATH="${SIM_ROOT}/models:${SIM_ROOT}/worlds:${GZ_SIM_RESOURCE_PATH:-}"
fi

exec "$@"
