#!/usr/bin/env bash
# Download the Gazebo actor meshes the world references, into the cache that
# compose mounts. Run once; later simulation runs then need no network.
#
# Intended to run inside the gazebo container.
set -euo pipefail

set +u
# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u

MODEL_URL="https://fuel.gazebosim.org/1.0/Mingfei/models/actor"

echo "Fetching ${MODEL_URL}"
gz fuel download -u "${MODEL_URL}"

MESHES="${HOME}/.gz/fuel/fuel.gazebosim.org/mingfei/models/actor"
if [ ! -d "${MESHES}" ]; then
    echo "FAIL: the actor model is not in the cache after downloading" >&2
    exit 1
fi

echo "Cached:"
find "${MESHES}" -name '*.dae' -printf '  %f\n' | sort
