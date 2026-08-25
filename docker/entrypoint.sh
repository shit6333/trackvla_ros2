#!/usr/bin/env bash
# Sources ROS 2 and, when present, the colcon workspace overlay before handing
# control to the requested command.
set -e

source "/opt/ros/${ROS_DISTRO}/setup.bash"

if [ -f /workspace/trackvla_ros2/install/setup.bash ]; then
    source /workspace/trackvla_ros2/install/setup.bash
fi

# The OmTrackVLA checkout is not a Python package; the adapter and the Phase 0
# gate import `model` and `cache_gridpool` from its root.
if [ -d "${OMTRACKVLA_SRC:-}" ]; then
    export PYTHONPATH="${OMTRACKVLA_SRC}:${PYTHONPATH:-}"
fi

exec "$@"
