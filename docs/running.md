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

### Step by step, with the GUI

`scripts/run_sim.sh` starts a headless simulator of its own, so it cannot be
used alongside a GUI Gazebo: the two would load the same world twice and
advertise the same topics. Start the pieces separately instead. Four shells,
two inside the VNC desktop and two on the host.

The VNC desktop runs inside the `vnc` container, so its terminals already have
the environment sourced and already see `/workspace/trackvla_ros2`. That
container is built from the Gazebo image and has no torch, so the inference
nodes cannot run there; they need `trackvla-ros2-dev`.

**A, in the VNC desktop.** The simulator:

```bash
cd /workspace/trackvla_ros2
gz sim -r sim/worlds/tracking.sdf
```

`-r` starts it running. Without it the world loads paused, every topic stays
silent, and the pipeline looks broken while nothing is wrong. If Gazebo cannot
resolve `model://turtlebot3_burger`, export the resource path first:

```bash
export GZ_SIM_RESOURCE_PATH=/workspace/trackvla_ros2/sim/models
```

**B, in the VNC desktop.** The bridge:

```bash
cd /workspace/trackvla_ros2
ros2 run ros_gz_bridge parameter_bridge --ros-args \
    -p config_file:=/workspace/trackvla_ros2/sim/config/bridge.yaml
```

Confirm before going further, because everything downstream depends on it:

```bash
ros2 topic hz /camera/image_raw     # about 10 Hz
```

**C, on the host.** The runtime nodes:

```bash
docker exec -it trackvla-ros2-dev bash
cd /workspace/trackvla_ros2
scripts/run_tracking.sh backend:=omtrackvla
```

Loading the checkpoint takes about 40 seconds. Wait for:

```text
backend 'omtrackvla' ready=True, inference rate 10.0 Hz
```

The executor also prints the limits it resolved, which is the cheapest way to
confirm a parameter edit was picked up:

```text
limits 1.00 m/s / 2.50 rad/s
```

**D, on the host.** The goal. Nothing before this step carries the
instruction: `run_tracking.sh` starts the nodes and they sit in IDLE until a
goal names a target, and no configuration file holds a default. Enter the
container through the entrypoint, which is what sources ROS and the workspace
overlay:

```bash
docker exec -it trackvla-ros2-dev /usr/local/bin/entrypoint.sh bash
```

Then, in that shell:

```bash
ros2 action send_goal /vla/track_target \
    vla_tracking_interfaces/action/TrackTarget \
    "{instruction: 'follow the person'}" --feedback
```

`--feedback` keeps printing status and never returns, because the task does not
end on its own; leave that terminal as a monitor or drop the flag.

To change the instruction, send another goal. It preempts the running one and
clears the backend's temporal state, so `predictions_published` restarts from
zero: this is a new task, not a renamed one.

`docker exec` bypasses the image's ENTRYPOINT, which is why a plain
`docker exec -it trackvla-ros2-dev bash` has no `ros2` on its PATH. Sourcing by
hand is equivalent as long as both lines are used:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash   # ros2, rclpy, the standard messages
source install/setup.bash                # vla_tracking_interfaces and the nodes
```

The first alone is not enough. `ros2` will run, but
`vla_tracking_interfaces/action/TrackTarget` will not resolve, and the goal
above fails on a type it cannot find.

### Checking a run

```bash
docker exec -it trackvla-ros2-dev bash -lc '
    source /opt/ros/$ROS_DISTRO/setup.bash; source install/setup.bash
    ros2 node list
    ros2 topic echo /vla/status --once | grep -E "task_state|predictions_published"
    ros2 topic hz /cmd_vel'
```

`task_state` 2 is TRACKING. A live node list is not proof of a live pipeline:
what matters is that `predictions_published` keeps climbing and `/cmd_vel` has
a rate. A node name appearing twice means a previous run was not fully stopped,
and two executors will be publishing competing commands.

### Stopping

Ctrl-C in C and D, then in A and B. Ctrl-C is what propagates the signal to a
launch file's children; killing the launch process alone leaves the nodes
orphaned and still publishing.

Do not stop them by name pattern. `pkill -f vla_inference_node` run through
`bash -lc` matches the shell's own command line, which contains that string, so
pkill kills itself before reaching the node. Take the PIDs first:

```bash
docker exec trackvla-ros2-dev bash -lc '
    PIDS=$(pgrep -f "lib/vla_tracking/")
    kill $PIDS; sleep 3; kill -9 $PIDS 2>/dev/null'
```

### When nothing moves

Two failures account for nearly all of them.

Gazebo is paused, because `-r` was omitted. `ros2 topic hz /camera/image_raw`
distinguishes this from every other cause in one command.

Or the person is outside the camera. The horizontal field of view is 90
degrees, so anything beyond 45 degrees off the nose does not exist as far as
the model is concerned, and an empty scene reads as "drive forward". The robot
spawns at the origin facing +x and the walker starts at (3, -2), a bearing of
-34 degrees, which is why that spawn pose is chosen rather than convenient.
Restarting Gazebo returns both to those positions.

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

Against the simulator, the same argument reaches the same node through the
sim launch:

```bash
scripts/run_tracking.sh backend:=omtrackvla enable_velocity_smoother:=true
```

Both are disabled by default and, when disabled, are absent rather than
configured to do nothing (D004).

The smoother turns a step change in commanded velocity into a ramp bounded by
`max_accel / smoothing_frequency`. That matters more in simulation than it
sounds: DiffDrive treats a command as a joint velocity target and reaches it
within one physics step, so an unsmoothed step is a step, and the TurtleBot3
Burger tipped forward onto its nose when the model commanded a stop at speed.
The limits in `config/velocity_smoother.yaml` are Nav2's defaults; the margin
they leave against this particular chassis is worked out in that file.

Confirm it is actually in the chain, since a disabled one is simply absent:

```bash
ros2 node list | grep velocity_smoother
ros2 topic hz /cmd_vel_raw     # the executor now publishes here
ros2 topic hz /cmd_vel         # and the smoother owns this
```

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
