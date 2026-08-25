# Container and development environment

## Fixed requirements

- container-only development and execution;
- Ubuntu 24.04 Noble;
- ROS 2 Jazzy;
- NVIDIA GPU and CUDA are required;
- system Python 3.12 is the ROS Python interpreter;
- no CPU fallback and no Conda environment in the ROS container;
- a single development/runtime container initially.

ROS 2 Jazzy officially supports Ubuntu 24.04. ROS binary Python extensions must
use a compatible system interpreter; the ROS documentation warns that Conda is
likely to be incompatible with binary ROS Python packages.

References:

- [ROS 2 Jazzy Ubuntu installation](https://docs.ros.org/en/jazzy/Installation/Alternatives/Ubuntu-Install-Binary.html)
- [Using Python packages with ROS 2](https://docs.ros.org/en/jazzy/How-To-Guides/Using-Python-Packages.html)

## Initial container topology

One CUDA-enabled container contains multiple packages and processes:

```text
vla_tracking container
├── ROS 2 Jazzy and custom interfaces
├── vla_inference_node
│   └── OmTrackVLABackend
│       └── PyTorch model on CUDA
├── trajectory_executor_node
├── Nav2 Velocity Smoother       installed, disabled by default
└── Nav2 Collision Monitor       installed, disabled by default
```

An interface package is a build-time artifact, not a running service, so it does
not justify a separate container. Keeping model inference in the inference node
also avoids an extra RGB serialization and IPC boundary.

## Planned image construction

The Dockerfile will use a pinned NVIDIA CUDA/cuDNN development image for Ubuntu
24.04, then install:

1. ROS 2 Jazzy apt repository and `ros-jazzy-desktop`;
2. `ros-dev-tools`, rosdep, and colcon;
3. `cv_bridge` and image transport;
4. Nav2 Velocity Smoother and Collision Monitor packages;
5. a CUDA-enabled PyTorch build compatible with the selected base image;
6. pinned inference-only OmTrackVLA dependencies;
7. the colcon workspace dependencies.

The exact CUDA base tag and PyTorch wheel versions will be selected and recorded
together during the environment-compatibility spike. Floating `latest` tags and
unversioned core ML dependencies are not allowed.

The container must fail clearly when CUDA is unavailable:

```text
torch.cuda.is_available() must be true
```

It must not silently fall back to CPU.

## OmTrackVLA compatibility boundary

The upstream full environment uses Ubuntu 22.04, Python 3.9, Conda, and
Habitat-Sim. This ROS project does not reuse that full environment because Jazzy
uses Ubuntu 24.04 and Python 3.12.

The ROS inference image installs only the model-serving subset:

- PyTorch and CUDA runtime;
- Transformers and Hugging Face support;
- DINO/SigLIP vision dependencies;
- NumPy, Pillow, and OpenCV as required by inference;
- the OmTrackVLA checkpoint and adapter.

It intentionally excludes Habitat-Sim, Habitat-Lab, HM3D, MP3D, and evaluation
assets.

The first technical gate is loading the checkpoint and performing one
inference-only forward pass under Python 3.12. If that cannot be made compatible
without invasive upstream changes, the fallback is two containers:

```text
ROS Jazzy container, Python 3.12
        |
        | versioned IPC request/response
        v
CUDA model-server container, Python 3.9
```

The project-owned backend interface must allow a future remote implementation,
but remote serving is not part of the first implementation unless the
compatibility gate fails.

## Planned Compose behavior

`compose.yaml` will provide:

- `gpus: all`;
- host networking for ROS DDS during development;
- host IPC and an explicit shared-memory size;
- source bind mount at `/workspace/trackvla_ros2`;
- persistent Hugging Face and Torch caches;
- `ROS_DOMAIN_ID` propagation;
- NVIDIA compute, utility, and graphics capabilities;
- matching developer UID/GID where practical.

The image build installs dependencies. Source is bind-mounted for development,
and `colcon build --symlink-install` runs inside the container. `build/`,
`install/`, and `log/` stay local and ignored.

## Planned ROS dependencies

Core:

```text
ros-jazzy-desktop
ros-dev-tools
ros-jazzy-cv-bridge
ros-jazzy-image-transport
```

Installed but disabled by default:

```text
ros-jazzy-nav2-velocity-smoother
ros-jazzy-nav2-collision-monitor
```

Gazebo is intentionally omitted from the first package build. Later simulation
work should use the supported Jazzy pairing, Gazebo Harmonic through `ros_gz`.

Reference:

- [Gazebo and ROS compatibility](https://gazebosim.org/docs/latest/ros_installation/)

## Expected developer commands

After implementation:

```bash
git submodule update --init --recursive
docker compose build
docker compose run --rm vla_tracking bash
```

Inside the container:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro jazzy -r -y
colcon build --symlink-install
source install/setup.bash
ros2 launch vla_tracking tracking.launch.py
```
