#!/usr/bin/env bash
# Stop this project's containers, and only this project's.
#
# Names are matched against an explicit list rather than a pattern. A pattern
# like "trackvla" also matches the unrelated omtrackvla evaluation container,
# which is how that one got deleted once; it is a different project that
# happens to share a prefix.
#
# Also removes the one-off containers that `docker compose run` leaves behind.
# `docker compose down` does not touch those: it only knows about services
# started with `docker compose up`, so run-created containers accumulate
# silently.
set -euo pipefail

NAMED=(tracking sim vnc pipeline foxglove markers trajmarkers goal
       trackvla-ros2-dev trackvla-gazebo trackvla-gazebo-gui trackvla-gazebo-vnc)

# Existence is checked before removing, not inferred from the exit status:
# recent Docker treats `rm -f` on a missing container as success, so acting on
# the exit status alone reports every name as stopped every time.
stopped=0
for name in "${NAMED[@]}"; do
    if [ -n "$(docker ps -aq --filter "name=^${name}$")" ]; then
        docker rm -f "${name}" > /dev/null 2>&1
        echo "stopped ${name}"
        stopped=$((stopped + 1))
    fi
done

# One-off containers from `docker compose run`, which this project's compose
# file prefixes with its own directory name.
while read -r name; do
    [ -z "${name}" ] && continue
    docker rm -f "${name}" > /dev/null 2>&1
    echo "stopped ${name} (leftover from compose run)"
    stopped=$((stopped + 1))
done < <(docker ps -a --format '{{.Names}}' | grep -E '^trackvla_ros2-.*-run-' || true)

echo "${stopped} container(s) stopped."
echo
echo "Still running:"
docker ps --format '  {{.Names}}\t{{.Status}}'
