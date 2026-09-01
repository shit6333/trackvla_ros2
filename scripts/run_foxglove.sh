#!/usr/bin/env bash
# Serve the ROS graph to Foxglove over a WebSocket.
#
# Connect from any machine that can reach this host: open Foxglove Studio,
# choose "Open connection", "Foxglove WebSocket", and enter
#
#     ws://<this host>:8765
#
# Unlike RViz this needs no X server, which is what makes it work from another
# machine. It runs in the vla_tracking container because that is where the
# custom interface package is; without it Foxglove cannot decode
# /vla/trajectory or /vla/status.
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/.."

exec docker compose run --rm vla_tracking bash -lc '
set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source install/setup.bash
set -u
echo "Foxglove bridge on ws://$(hostname -I | awk "{print \$1}"):8765"
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
'
