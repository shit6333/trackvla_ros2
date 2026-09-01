#!/usr/bin/env bash
# Open the Gazebo GUI on the desktop session, attached to a simulator that is
# already running headless.
#
# Requires a live desktop session. DISPLAY_ID and HOST_RUNTIME_DIR in .env must
# match it: run `echo $DISPLAY` in a terminal inside that session, and
# HOST_RUNTIME_DIR is normally /run/user/<your uid>.
#
# Rendering is software, so expect a usable but sluggish window; see the
# gazebo-gui service in compose.yaml for why.
set -euo pipefail

cd "$(dirname "${BASH_SOURCE[0]}")/.."
exec docker compose run --rm gazebo-gui "$@"
