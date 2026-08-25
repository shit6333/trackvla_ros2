#!/usr/bin/env bash
# Resolve declared dependencies and build the colcon workspace.
# Intended to run inside the container, from the workspace root.
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/.."

if [ ! -d /opt/ros/"${ROS_DISTRO}" ]; then
    echo "FAIL: /opt/ros/${ROS_DISTRO} not found; run this inside the container" >&2
    exit 1
fi

# ROS's setup scripts read unset variables, so nounset must be relaxed while
# they are sourced.
set +u
# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u

# rosdep's index is not baked into the image, so refresh it on first use. A
# refresh failure is not fatal when the cache is already present.
if [ ! -d "${HOME}/.ros/rosdep" ]; then
    rosdep update || echo "warning: rosdep update failed; using any cached index" >&2
fi

rosdep install \
    --from-paths src \
    --ignore-src \
    --rosdistro "${ROS_DISTRO}" \
    -r -y

colcon build --symlink-install "$@"

echo
echo "Build complete. Source the overlay with:"
echo "    source install/setup.bash"
