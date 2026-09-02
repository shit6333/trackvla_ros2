#!/usr/bin/env bash
# Bring up the tracking pipeline against the simulator, with the viewers.
#
# The simulator itself runs separately, in the gazebo container:
#
#     docker compose run -d --name sim gazebo ./scripts/run_sim.sh
#     docker compose run -d --name tracking vla_tracking ./scripts/run_tracking.sh
#
# Then send a goal into the running container rather than starting another:
#
#     docker exec tracking bash -lc 'source install/setup.bash && \
#         ros2 action send_goal /vla/track_target \
#         vla_tracking_interfaces/action/TrackTarget \
#         "{instruction: '\''follow the person'\''}"'
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/.."

set +u
# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source install/setup.bash
set -u

exec ros2 launch sim/launch/tracking_sim.launch.py "$@"
