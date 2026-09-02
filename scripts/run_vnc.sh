#!/usr/bin/env bash
# Start a display inside the container and serve it over VNC.
#
# Xvnc is an X server, not a screen-sharing tool: this display exists only in
# this container and is not connected to the host's desktop, so an application
# that crashes here cannot take the host's session with it. That is why it
# exists; the Gazebo GUI crashed GNOME Shell every time it was pointed at the
# real session.
#
# The server binds to localhost by default, so reach it through an SSH tunnel
# rather than exposing an unauthenticated display on the network:
#
#     ssh -L 5901:localhost:5901 <this host>
#
# then connect a VNC client to localhost:5901. On macOS, Finder's
# Go > Connect to Server with vnc://localhost:5901 works.
#
# A password is set because macOS Screen Sharing refuses a server that offers
# no authentication, and it is the client most likely to be used here. It
# defaults to "trackvla" and can be changed with VNC_PASSWORD. Classic VNC
# authentication truncates at eight characters and is weak, which is why the
# server still binds to localhost: the SSH tunnel is the actual protection.
#
# Pass --public to bind every interface instead. Do that only on a network you
# trust.
#
# The display number defaults to :20 rather than :1 because this container
# runs with host networking, and an X server's abstract unix socket lives in
# the network namespace. Low numbers collide with the host's own displays,
# including other users', and Xvnc then refuses to start with "server already
# running". Override with VNC_DISPLAY.
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/.."

DISPLAY_NUMBER="${VNC_DISPLAY:-:20}"
GEOMETRY="${VNC_GEOMETRY:-1600x900}"
LOCALHOST=1
for arg in "$@"; do
    [ "${arg}" = "--public" ] && LOCALHOST=0
done

set +u
# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u

PASSWORD="${VNC_PASSWORD:-trackvla}"
PASSWORD_FILE=/tmp/vncpasswd
# x11vnc is used only as a password-file generator; this build of
# tigervnc ships no vncpasswd.
x11vnc -storepasswd "${PASSWORD}" "${PASSWORD_FILE}" > /dev/null 2>&1
chmod 600 "${PASSWORD_FILE}"

echo "Starting Xvnc on ${DISPLAY_NUMBER} at ${GEOMETRY}"
Xvnc "${DISPLAY_NUMBER}" \
    -geometry "${GEOMETRY}" \
    -depth 24 \
    -SecurityTypes VncAuth \
    -rfbauth "${PASSWORD_FILE}" \
    -localhost="${LOCALHOST}" \
    -AlwaysShared \
    > /tmp/xvnc.log 2>&1 &
XVNC_PID=$!

for _ in $(seq 20); do
    DISPLAY="${DISPLAY_NUMBER}" xdpyinfo > /dev/null 2>&1 && break
    sleep 0.5
done
if ! DISPLAY="${DISPLAY_NUMBER}" xdpyinfo > /dev/null 2>&1; then
    echo "FAIL: Xvnc did not come up" >&2
    tail -20 /tmp/xvnc.log >&2
    exit 1
fi

export DISPLAY="${DISPLAY_NUMBER}"
openbox > /tmp/openbox.log 2>&1 &
xterm -geometry 100x28+40+40 > /tmp/xterm.log 2>&1 &

PORT=$((5900 + ${DISPLAY_NUMBER#:}))
echo
echo "Display ${DISPLAY_NUMBER} is up, served on port ${PORT}."
if [ "${LOCALHOST}" = "1" ]; then
    echo "  ssh -L ${PORT}:localhost:${PORT} <this host>"
    echo "  then connect a VNC client to localhost:${PORT}"
    echo "  password: ${PASSWORD}"
else
    echo "  connect a VNC client to <this host>:${PORT}"
    echo "  password: ${PASSWORD}"
fi
echo
echo "Inside the session, or with docker exec:"
echo "  DISPLAY=${DISPLAY_NUMBER} gz sim sim/worlds/tracking.sdf"
echo

wait "${XVNC_PID}"
