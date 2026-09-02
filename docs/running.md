# Building and running

All development and execution happens inside the container. Nothing here
should be run on the host.

## First time

```bash
git clone <this repository> && cd trackvla_ros2
git submodule update --init --recursive

cp .env.example .env
# Edit .env: MODEL_CACHE must point at a directory containing the Hugging Face
# cache and the OmTrackVLA checkpoint. USER_ID and GROUP_ID should match your
# host user so bind-mounted files stay writable.

docker compose build
```

## Verifying the environment

```bash
docker compose run --rm vla_tracking ./scripts/check_gpu.sh
docker compose run --rm vla_tracking python3 scripts/phase0_check.py
```

`check_gpu.sh` fails if CUDA is unavailable or if this PyTorch build has no
kernels for the installed GPU. `phase0_check.py` additionally loads the
checkpoint and runs one forward pass.

## Building the workspace

```bash
docker compose run --rm vla_tracking ./scripts/build_workspace.sh
```

This runs `rosdep install` and `colcon build --symlink-install`. Because of
`--symlink-install`, editing a `.py` file takes effect without rebuilding;
editing a `.msg` or `.action` file does require a rebuild, since those are
compiled.

## Running

Against the simulator, two containers:

```bash
docker compose run -d --name sim      gazebo       ./scripts/run_sim.sh
docker compose run -d --name tracking vla_tracking ./scripts/run_tracking.sh
```

The second brings up both runtime nodes, the Foxglove bridge, and the marker
publishers together. They all join one ROS graph, and with host networking a
separate container buys a node no isolation it does not already have; it only
adds something else to remember to stop.

Send a goal into the running container rather than starting another:

```bash
docker exec tracking bash -lc 'source install/setup.bash && \
    ros2 action send_goal /vla/track_target \
    vla_tracking_interfaces/action/TrackTarget \
    "{instruction: '"'"'follow the person'"'"'}"'
```

Stop with `docker rm -f tracking sim`. Name them; a pattern like `trackvla`
also matches the unrelated OmTrackVLA evaluation container.

Without a simulator, the pipeline alone:

```bash
docker compose run --rm vla_tracking bash
source install/setup.bash
ros2 launch vla_tracking tracking.launch.py                    # fake backend
ros2 launch vla_tracking tracking.launch.py backend:=omtrackvla
```

## Watching a run

Foxglove Studio connects over a WebSocket, so it needs no display and works
from another machine:

```
ws://<this host>:8765
```

Set the 3D panel's display frame to `tracking`, then add `/scene_markers` and
`/vla/trajectory_markers`. Images are on `/camera/image_raw`, which is what
the model sees, and `/sim/third_person/image_raw`.

Two things that view cannot show. The walking person is absent because Gazebo
publishes no pose for an actor at all. And `tracking` is ground truth from a
simulator plugin, not odometry: `/odom` drifts, measured at 3.2 m after a few
minutes, so a scene drawn against it sits metres from the robot.

## The Gazebo GUI

Run it on a display of the container's own, not the host's desktop:

```bash
docker compose run -d --name vnc gazebo-vnc
```

Then type in the xterm inside the VNC session. It already has the ROS
environment and `GZ_SIM_RESOURCE_PATH`, so nothing needs sourcing:

```bash
gz sim sim/worlds/tracking.sdf     # server and GUI together, for editing
gz sim -g                          # GUI only, attached to a running server
```

The difference is `-g`. Without it Gazebo starts its own server, which is what
you want while editing a world: close it, edit the SDF, open it again. With
it, the GUI attaches to the headless server in the `sim` container, which is
how to watch the pipeline drive the robot.

The Entity Tree and Component Inspector panels are the fastest way to see what
a piece of SDF actually became, which beats reading the file and guessing.

Reach it through an SSH tunnel, since the server has only a weak VNC password
and binds to localhost:

```bash
ssh -L 5920:localhost:5920 <this host>
# ssh -L 5920:localhost:5920 y_ricky@192.168.1.108
# then connect a VNC client to localhost:5920, password trackvla
```

Pointing the GUI at the host's desktop session instead crashes GNOME Shell and
takes the remote desktop down with it; `compose.yaml` records the diagnosis
beside the `gazebo-gui` service. Rendering here is software, which is fast
enough to inspect a scene, and does not affect the simulation itself: physics
and the camera sensors still run on the GPU in the headless server.

This is also the only view that shows the walking person. Gazebo renders
actors but publishes no pose for them, so they reach the GUI and never reach
Foxglove.

Then, from a second shell into the same container:

```bash
ros2 action send_goal /vla/track_target \
    vla_tracking_interfaces/action/TrackTarget \
    "{instruction: 'follow the person in the red shirt'}" --feedback
```

The pipeline expects a camera on `/camera/image_raw`. With no camera the nodes
run and report status, but no trajectory is ever published.

### Optional Nav2 chain

```bash
ros2 launch vla_tracking tracking.launch.py enable_velocity_smoother:=true
ros2 launch vla_tracking tracking.launch.py enable_collision_monitor:=true
```

Both are disabled by default and, when disabled, are absent rather than
configured to do nothing (D004).

Enabling the collision monitor requires a real sensor on its configured source
topic. It is fail-safe, so with no data arriving it holds the base stopped once
`source_timeout` elapses. Its stop polygon in
`config/collision_monitor.yaml` is a placeholder that must be replaced with the
real robot footprint before any hardware use.

## Tests

```bash
docker compose run --rm vla_tracking bash -lc '
    source install/setup.bash && colcon test && colcon test-result --verbose'
```

Tests that need CUDA and the checkpoint skip cleanly when either is absent, so
the suite runs on a machine with no GPU. Everything else uses the `fake`
backend and downloads no weights.

## Recording data

Record the model-facing interfaces plus the commands they produced. Camera
frames dominate the file size, so record them only when the imagery itself is
under study.

```bash
# Behaviour only: small, enough to replay decisions against commands
ros2 bag record /vla/trajectory /vla/status /cmd_vel /vla/track_target/_action/status

# With imagery: large. Prefer compressed transports if the camera offers them.
ros2 bag record /camera/image_raw /vla/trajectory /vla/status /cmd_vel
```

When the Nav2 chain is enabled, also record `/cmd_vel_raw` and, with both
stages on, `/cmd_vel_smoothed`, so that a command can be traced from the model
through each stage to the base.

Every trajectory carries the observation timestamp it was predicted from, so a
bag is enough to align a decision with the frame that caused it without
recording the frames themselves.

## Provenance

The exact upstream and model versions this workspace was verified against are
recorded in [`docs/provenance.md`](provenance.md).
