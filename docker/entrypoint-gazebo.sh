#!/usr/bin/env bash
# Sources ROS and points Gazebo at the simulation assets in this repository.
set -e

source "/opt/ros/${ROS_DISTRO}/setup.bash"

SIM_ROOT=/workspace/trackvla_ros2/sim
if [ -d "${SIM_ROOT}/models" ]; then
    export GZ_SIM_RESOURCE_PATH="${SIM_ROOT}/models:${SIM_ROOT}/worlds:${GZ_SIM_RESOURCE_PATH:-}"
fi

# GNOME Remote Desktop names its Xwayland cookie with a random suffix and
# writes a new one on every reconnect, so it is discovered at start rather
# than configured. Without it an X client is refused with "Authorization
# required, but no authorization protocol specified".
if [ -z "${XAUTHORITY:-}" ] && [ -d "${HOST_RUNTIME_DIR:-}" ]; then
    cookie=$(ls -t "${HOST_RUNTIME_DIR}"/.mutter-Xwaylandauth.* 2>/dev/null | head -1)
    if [ -n "${cookie}" ]; then
        export XAUTHORITY="${cookie}"
        echo "[entrypoint] using X cookie ${cookie}"
    else
        echo "[entrypoint] no Xwayland cookie under ${HOST_RUNTIME_DIR};" \
             "GUI clients will be refused" >&2
    fi
fi

# Qt writes here and warns loudly when it is unset. The host's runtime
# directory is mounted read-only, so it cannot be reused.
if [ -n "${XDG_RUNTIME_DIR:-}" ]; then
    mkdir -p "${XDG_RUNTIME_DIR}" && chmod 700 "${XDG_RUNTIME_DIR}"
fi

# Refuse early, and legibly, when the display is not there. A remote desktop
# session takes its X server with it when it disconnects, leaving the socket
# behind, so a container started afterwards connects to nothing and Qt aborts
# with a stack trace that says only that platform integration failed. Stale
# cookies accumulate for the same reason, which is why the newest is chosen
# above rather than any that happens to exist.
if [ -n "${DISPLAY:-}" ]; then
    # Strip the screen suffix from the display number, not from the path:
    # /tmp/.X11-unix already contains a dot, so trimming the whole path at the
    # first one leaves "/tmp/." and the check reports a nonsense location.
    display_number="${DISPLAY#:}"
    display_number="${display_number%%.*}"
    socket="/tmp/.X11-unix/X${display_number}"
    if [ ! -S "${socket}" ]; then
        echo "[entrypoint] DISPLAY=${DISPLAY} has no socket at ${socket}." >&2
        echo "[entrypoint] Is a desktop session running? Check DISPLAY_ID in .env." >&2
    elif ! timeout 5 python3 -c "
import socket, sys
s = socket.socket(socket.AF_UNIX)
try:
    s.connect(sys.argv[1])
except OSError as exc:
    raise SystemExit(f'{exc}')
" "${socket}" 2>/dev/null; then
        echo "[entrypoint] ${socket} exists but refuses connections." >&2
        echo "[entrypoint] The session that owned it has probably ended;" >&2
        echo "[entrypoint] reconnect the remote desktop and try again." >&2
    fi
fi

exec "$@"
