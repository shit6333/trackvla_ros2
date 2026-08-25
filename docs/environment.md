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

## Reference environment

Version selection is anchored to the OmTrackVLA evaluation container
(`omtrackvla-dev`), whose runtime produced the corrected EVT-Bench results. Its
installed modules were measured on 2026-08-25:

| Module | OmTrackVLA runtime | This image | Note |
| --- | --- | --- | --- |
| Python | 3.9.19 (conda) | 3.12 (system) | ROS 2 Jazzy interpreter |
| torch | 2.8.0+cu128 | 2.8.0+cu128 | identical |
| torchvision | 0.23.0+cu128 | 0.23.0+cu128 | identical |
| CUDA / cuDNN | 12.8 / 9.10.2 | 12.8 / 9.10.2 | identical base toolchain |
| transformers | 4.57.6 | 4.57.6 | checkpoint declares 4.57.3 |
| huggingface_hub | 0.36.2 | 0.36.2 | identical |
| accelerate | 1.10.1 | 1.10.1 | identical |
| safetensors | 0.7.0 | 0.7.0 | identical |
| einops | 0.8.2 | 0.8.2 | identical |
| timm | 1.0.28 | 1.0.28 | identical |
| pillow | 10.3.0 | 10.3.0 | identical |
| scipy | 1.13.1 | 1.11.4 | deviation, see below |
| numpy | 1.23.5 | 1.26.4 | deviation, see below |
| opencv-python-headless | 4.8.1.78 | not installed | deviation, see below |
| habitat-sim | 0.3.1 | not installed | inference does not use it |

### Deviation: numpy

OmTrackVLA pins `numpy==1.23.5` for habitat-sim. That version has no CPython
3.12 wheel, and ROS 2 Jazzy's binary Python extensions link against the distro
`python3-numpy`, which is 1.26.4 on Ubuntu 24.04. Installing a second numpy
would risk two ABIs in one interpreter.

The inference path (`model.py`, `cache_gridpool.py`, `open_trackvla_hf/`) uses
no numpy alias removed in 1.24, so this image tracks the distro version.

### Deviation: scipy

OmTrackVLA pins `scipy==1.13.1`, but `ros-jazzy-desktop` pulls in
`python3-scipy` 1.11.4 and pip cannot uninstall a dpkg-installed package (it
carries no `RECORD` file). The inference path imports no scipy, so the distro
version is used, as with numpy.

### Deviation: OpenCV

OmTrackVLA pins `opencv-python-headless==4.8.1.78`, which also has no CPython
3.12 wheel. The inference path imports no `cv2` at all, and ROS image
conversion is handled by `ros-jazzy-cv-bridge` from apt. Installing a pip
OpenCV alongside it would only introduce an ABI conflict, so none is installed.

## Image construction

Base image:

```text
nvidia/cuda:12.8.1-cudnn-devel-ubuntu24.04
```

This is the same CUDA and cuDNN toolchain as the OmTrackVLA container, rebased
onto Noble. CUDA 12.8 is not optional: the target GPU is an RTX PRO 6000
Blackwell (compute capability `sm_120`), and `torch==2.8.0+cu128` is the pinned
build whose kernel list contains `sm_120`.

Build stages:

1. locale, base tooling;
2. ROS 2 Jazzy apt repository via `ros2-apt-source` **pinned to 1.2.0** — the
   upstream instructions resolve this through the GitHub "latest release" API,
   which is a floating reference;
3. `ros-jazzy-desktop`, `ros-dev-tools`, colcon, rosdep, vcstool;
4. `ros-jazzy-cv-bridge`, `ros-jazzy-image-transport`, `ros-jazzy-vision-opencv`;
5. `ros-jazzy-nav2-velocity-smoother`, `ros-jazzy-nav2-collision-monitor`;
6. `requirements/torch-cu128.txt`;
7. `requirements/inference.txt`.

Ubuntu 24.04 marks the system interpreter as externally managed (PEP 668).
`rclpy` lives in that interpreter, so the inference stack is installed beside it
with `PIP_BREAK_SYSTEM_PACKAGES=1` rather than in an isolated virtual
environment. A venv would either hide `rclpy` from torch or hide torch from
`rclpy`.

The container must fail clearly when CUDA is unavailable:

```text
torch.cuda.is_available() must be true
```

It must not silently fall back to CPU. `scripts/check_gpu.sh` enforces this and
additionally asserts that the device's compute capability appears in
`torch.cuda.get_arch_list()`.

## Phase 0 gate result

The compatibility gate passed on 2026-08-25, 8/8 stages
(`scripts/phase0_check.py`, image `trackvla-ros2:jazzy-cu128`):

| Stage | Result |
| --- | --- |
| `rclpy` and torch in one interpreter | Python 3.12.3, node creation succeeds |
| CUDA device visible | RTX PRO 6000 Blackwell, `sm_120` present in the arch list |
| Upstream inference modules import | no `habitat` module pulled in |
| Vision encoders | DINOv3 + SigLIP, 24x24 grid, coarse `(4, 1536)` / fine `(64, 1536)` |
| Checkpoint load | `OmTrackVLA-0.6B`, 609.2M parameters |
| Forward pass | finite `(1, 8, 3)`, explicit `dt = 0.1 s` |
| Reset isolation | a second frame changes the output; clearing history reproduces the first prediction bit-for-bit |

The Python 3.12 compatibility gate therefore **passed**, so the single-container
design in D005 stands and the two-container model-server fallback below is not
activated. Resolved versions are recorded in `requirements/inference-lock.txt`.

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
