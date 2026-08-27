# Implementation plan

## Goal

Deliver a containerized ROS 2 Jazzy package that accepts monocular RGB history
and a natural-language tracking instruction, invokes a replaceable VLA backend,
publishes a well-defined short-horizon trajectory, and converts a configurable
number of predicted waypoints to base velocity commands.

Gazebo and real-robot integration begin only after the package-level pipeline is
built and tested.

## Development and commit workflow

- Work is implemented in small, independently verifiable increments.
- Each completed phase or coherent sub-phase is tested before a local commit.
- Commits stay on the local repository until an online repository is created.
- Do not configure a remote or push without the project owner's repository URL
  and explicit direction.
- Build artifacts, caches, checkpoints, credentials, and local `.env` files are
  never committed.

## Phase 0 — Environment compatibility spike

Deliverables:

- pinned Ubuntu 24.04 CUDA/cuDNN base image;
- ROS 2 Jazzy installed and sourced;
- CUDA-enabled PyTorch with `torch.cuda.is_available() == True`;
- inference-only OmTrackVLA dependencies under system Python 3.12;
- successful checkpoint load and one representative forward pass;
- recorded versions for CUDA, PyTorch, Transformers, vision encoders, and model
  revision.

Exit criteria:

- `rclpy` and PyTorch import in the same interpreter;
- the GPU is visible inside the container;
- the backend returns finite `[N, 3]` waypoints and an explicit `dt`;
- repeated reset/inference does not retain a prior task's temporal state;
- if Python 3.12 is blocked, document the exact incompatibility and activate the
  remote model-server fallback design before continuing.

## Phase 1 — Workspace and ROS interfaces

Create the three initial packages:

```text
vla_tracking_interfaces
vla_tracking
vla_tracking_omtrackvla
```

Deliverables:

- valid `package.xml`, `CMakeLists.txt`, `setup.py`, and `setup.cfg` files;
- `VlaTrajectory.msg` with frame, timestamp, waypoint array, `dt`, backend, and
  validity fields;
- `VlaStatus.msg` with task/backend state and inference timing;
- `TrackTarget.action` with instruction goal, state feedback, and result;
- interface generation and import tests;
- container scripts for rosdep and `colcon build --symlink-install`.

Exit criteria:

- clean build from a fresh container;
- custom message and action types can be inspected with `ros2 interface show`;
- unit tests and `colcon test-result --verbose` pass.

## Phase 2 — Backend contract and OmTrackVLA adapter

Deliverables:

- project-owned backend protocol and normalized observation/prediction types;
- backend loader selected by ROS parameter;
- OmTrackVLA adapter using the pinned upstream source and checkpoint;
- RGB conversion and normalization with explicit encoding handling;
- temporal history initialization, append, reset, and task-switch behavior;
- output conversion to metres, radians, `base_link`, and explicit `dt`;
- a fake backend for fast node and integration tests without model weights.

Exit criteria:

- generic code has no import dependency on OmTrackVLA internals;
- local fake and OmTrackVLA backends satisfy the same contract;
- fixed test inputs give deterministic shape, frame, and timing metadata;
- task reset removes all previous history and predicted-trajectory state.

## Phase 3 — `vla_inference_node`

Deliverables:

- `TrackTarget` action server;
- sensor-data QoS image subscription;
- latest-frame handling in the node; the temporal history itself belongs
  to the backend (see D012);
- inference worker/timer separated from image callbacks;
- prevention of overlapping inference jobs;
- trajectory, status, and optional debug-image publishers;
- cancellation, model failure, stale-image, and task-switch handling;
- ROS parameters for backend, model ID/path, image topic, history, frames, and
  inference scheduling.

Exit criteria:

- fake camera plus fake backend exercises the entire ROS action/topic path;
- a new instruction preempts and resets the previous task;
- image callbacks remain responsive while inference runs;
- timestamps on predictions identify the image used for inference;
- no trajectory is published for an inactive or stale task.

## Phase 4 — `trajectory_executor_node`

Deliverables:

- trajectory subscription and fixed-rate command timer;
- validation for shape, finite values, frame, `dt`, timestamp, and physical hard
  limits;
- conversion of consecutive `[x, y, theta]` poses to timed velocity segments;
- configurable `waypoints_to_execute`, default `1`;
- immediate preemption on every newer valid trajectory;
- zero velocity on completion, timeout, cancellation, invalid data, or shutdown;
- differential-drive conversion/configuration for the initial target base;
- unit tests using synthetic straight, turning, preempted, stale, and invalid
  trajectories.

Exit criteria:

- `waypoints_to_execute: 1` matches the released OmTrackVLA evaluator's
  first-executable-waypoint behavior;
- larger values execute no more than the requested count and are preemptible;
- missing new trajectories cannot leave a non-zero command latched;
- NaN, invalid `dt`, and excessive commands never reach `/cmd_vel`;
- no odometry dependency exists in the first open-loop implementation.

## Phase 5 — Launch, configuration, and optional Nav2 nodes

Deliverables:

- `tracking.launch.py` for inference plus executor;
- one readable `tracking.yaml` for core parameters;
- optional Velocity Smoother and Collision Monitor YAML files;
- launch arguments defaulting both Nav2 nodes to disabled;
- correct topic routing for all four enabled/disabled combinations;
- startup CUDA/model health reporting.

Exit criteria:

- exactly one final publisher controls `/cmd_vel` in every launch combination;
- raw mode does not subscribe to scan/depth or apply Nav2 smoothing;
- Nav2-disabled operation requires no Nav2 sensor input;
- launch shutdown publishes or causes a final zero command.

## Phase 6 — Package-level integration and documentation

Deliverables:

- integration test publisher for RGB images;
- action client fixture for target instructions;
- trajectory and command observers;
- container build/run documentation;
- topic/interface documentation and example launch parameters;
- rosbag recording recommendations;
- pinned upstream model revision and dependency manifest.

Exit criteria:

- a fresh checkout and submodule initialization builds only through documented
  container commands;
- fake-backend integration tests run without downloading weights;
- GPU-backed OmTrackVLA smoke test runs when the checkpoint/cache is available;
- lint, unit tests, integration tests, and `git diff --check` pass.

## Deferred work

After the package-level exit criteria pass:

- Gazebo Harmonic world and robot integration;
- camera/FOV/domain alignment experiments;
- obstacle and distractor scenarios;
- optional Nav2 safety ablations;
- closed-loop odometry-feedback trajectory control;
- physical robot driver and sensor remapping;
- support for additional fine-tuned tracking or approach backends.

## Initial parameters

Provisional core configuration:

```yaml
vla_inference_node:
  ros__parameters:
    backend: omtrackvla        # entry point name, not a class path
    image_topic: /camera/image_raw
    base_frame: base_link
    inference_rate: 10.0       # Hz; matches the checkpoint's 0.1 s cadence
    max_image_age: 0.5         # seconds; 0 disables the staleness check
    status_rate: 5.0
    model_dir: ''              # empty falls back to HF_MODEL_DIR
    source_path: ''            # empty falls back to OMTRACKVLA_SRC
    require_cuda: true
    history_length: 0          # 0 keeps the backend's own default of 31

trajectory_executor_node:
  ros__parameters:
    waypoints_to_execute: 1
    trajectory_timeout: 0.3
    max_linear_velocity: 0.2
    max_angular_velocity: 0.5
```

The numerical rate, timeout, velocity, and acceleration values are placeholders
until checkpoint timing and target robot limits are measured. They must not be
treated as validated robot settings.

## Open decisions before simulator integration

- target robot kinematics and command interface;
- whether lateral model velocity is projected, rejected, or supported;
- verified checkpoint coordinate convention and waypoint timing;
- camera resolution, FOV, and history sampling cadence;
- inference rate achievable on the target GPU;
- criteria for adding closed-loop odometry feedback;
- Gazebo robot, target-person assets, and evaluation scenarios.
