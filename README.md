# TrackVLA ROS 2

ROS 2 integration for vision-language tracking and navigation models. The first
backend will be OmTrackVLA, but the ROS interfaces and runtime nodes are designed
so that compatible tracking or approach models can be substituted later.

## Current status

Phases 0 to 2 are complete. The CUDA + ROS 2 Jazzy image builds, the three ROS
packages build and pass their tests, and the OmTrackVLA planner is reachable
through a model-independent backend contract. Steady-state inference measures
39 ms, about 25.6 Hz. The runtime nodes themselves do not exist yet; they
arrive in Phases 3 and 4.

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
