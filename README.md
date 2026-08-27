# TrackVLA ROS 2

ROS 2 integration for vision-language tracking and navigation models. The first
backend will be OmTrackVLA, but the ROS interfaces and runtime nodes are designed
so that compatible tracking or approach models can be substituted later.

## Current status

Phases 0 to 5 are complete. One launch command brings up the pipeline: an
instruction starts a task, camera frames drive inference, trajectories are
published, and the executor converts them into `/cmd_vel`. In-pipeline
OmTrackVLA inference measures 46.6 ms. Phase 6, integration testing and
documentation, is what remains.

```bash
ros2 launch vla_tracking tracking.launch.py
ros2 launch vla_tracking tracking.launch.py backend:=omtrackvla

ros2 action send_goal /vla/track_target \
    vla_tracking_interfaces/action/TrackTarget \
    "{instruction: 'follow the person in the red shirt'}" --feedback
```

The Nav2 velocity smoother and collision monitor are installed but disabled;
with both off the executor publishes `/cmd_vel` directly rather than passing
through no-op stages.

```bash
ros2 launch vla_tracking tracking.launch.py enable_velocity_smoother:=true
ros2 launch vla_tracking tracking.launch.py enable_collision_monitor:=true
```

Enabling the collision monitor requires a real sensor on its source topic. It
is fail-safe by design, so with no data arriving it holds the base stopped.

**The default velocity limits are not validated against any robot, and the
linear limit is too low for this checkpoint.** The model predicts roughly
0.49 m/s forward against a 0.2 m/s limit, so the clamp is engaged
continuously and the executor discards the model's speed control. See
`config/tracking.yaml`.

```bash
cp .env.example .env      # then set MODEL_CACHE
git submodule update --init --recursive
docker compose build

# Phase 0 gate
docker compose run --rm vla_tracking ./scripts/check_gpu.sh
docker compose run --rm vla_tracking python3 scripts/phase0_check.py

# Phases 1 and 2: workspace and backends
docker compose run --rm vla_tracking ./scripts/build_workspace.sh
```

The OmTrackVLA adapter tests need CUDA and a checkpoint at `HF_MODEL_DIR`;
they skip cleanly when either is absent.

## Agreed first version

- Ubuntu 24.04, ROS 2 Jazzy, NVIDIA CUDA, and container-only development.
- One CUDA-enabled development container initially.
- Two required runtime nodes:
  - `vla_inference_node`
  - `trajectory_executor_node`
- A model-independent backend interface with an OmTrackVLA adapter.
- The model publishes a timestamped short-horizon trajectory.
- `trajectory_executor_node` executes at most a configurable number of future
  waypoints from each prediction; the default is one.
- A newer prediction always preempts the remaining points of the older one.
- Nav2 Velocity Smoother and Collision Monitor are retained as optional launch
  components and are disabled by default.
- Gazebo and physical-robot integration are deferred until the packages build
  and the topic pipeline is validated.

## Planning documents

- [System architecture](docs/architecture.md)
- [Repository and package layout](docs/repository-layout.md)
- [Container and development environment](docs/environment.md)
- [Implementation plan](docs/implementation-plan.md)
- [Recorded design decisions](docs/decisions.md)

## Upstream references

- [OmTrackVLA](https://github.com/om-ai-lab/OmTrackVLA)
- [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/)
- [Navigation2](https://docs.nav2.org/)

## Git workflow

Development is local-first. Completed, verified increments are committed to the
local `main` branch. No remote is configured and nothing is pushed until the
project owner creates the online repository and provides its URL.
