# TrackVLA ROS 2

ROS 2 integration for vision-language tracking and navigation models. The first
backend will be OmTrackVLA, but the ROS interfaces and runtime nodes are designed
so that compatible tracking or approach models can be substituted later.

## Current status

Planning only. No ROS packages, model runtime, or container image have been
implemented yet.

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
