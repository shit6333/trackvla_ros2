# Deploying on a robot

Written 2026-09-08. Nothing in this document has run on hardware. The
pipeline has only ever driven the Gazebo TurtleBot on the workstation
described in [provenance.md](provenance.md); every number below that came
from a measurement says where it was measured, and every step that has not
been exercised says so. Treat this as the shortest path to a first run, not as
a validated procedure.

## What a clone does not contain

The repository holds the ROS packages, the container definitions, the
simulator assets and these documents. Four things it needs are outside git:

| Missing after `git clone` | How to get it |
| --- | --- |
| `third_party/OmTrackVLA` (submodule, pinned) | `git submodule update --init --recursive` |
| Model weights: `OmTrackVLA-0.6B/` and the Hugging Face cache with Qwen3-0.6B, DINOv3 and SigLIP (6.0 GB together) | unpack a weights tarball (below) or copy the whole `cache/` directory from a machine that has it. DINOv3 is a gated download, so do not rely on the hub fetching it on a fresh machine |
| `.env` | `cp .env.example .env`, then set `MODEL_CACHE`, `USER_ID`/`GROUP_ID` (from `id -u` / `id -g`) and `ROS_DOMAIN_ID` |
| The inference image (33 GB) and the colcon `install/` tree | `docker compose build vla_tracking`, then `./scripts/build_workspace.sh` inside it |

The `gazebo` services in `compose.yaml` are not needed on a robot; build only
`vla_tracking`. Without internet, move the image with `docker save` /
`docker load` instead of rebuilding.

### The weights tarball

The backend reads exactly these files, and nothing else in `cache/`:

    OmTrackVLA-0.6B/          config.json, checkpoint_meta.json, model.safetensors, the three .py files
    huggingface/hub/          models--Qwen--Qwen3-0.6B, models--google--siglip-so400m-patch14-384,
                              models--facebook--dinov3-vits16-pretrain-lvd1689m

`pytorch_model.bin` in the checkpoint directory is not read when
`model.safetensors` is present (verified 2026-09-10: removing it leaves the
forward pass bit-identical), and the Hugging Face `token` file must never
travel with the weights. The command that produces a tarball with the right
contents, run on a machine that has `cache/`:

```bash
cd /path/to/cache
tar -cf omtrackvla_weights.tar \
    --exclude='OmTrackVLA-0.6B/pytorch_model.bin' --exclude='OmTrackVLA-0.6B/*.gif' \
    --exclude='OmTrackVLA-0.6B/.cache' --exclude='huggingface/token' \
    --exclude='huggingface/stored_tokens' --exclude='huggingface/xet' \
    --exclude='huggingface/hub/.locks' --exclude='huggingface/hub/models--Qwen--Qwen3-4B' \
    --exclude='huggingface/hub/models--omlab--OmTrackVLA-0.6B' \
    OmTrackVLA-0.6B huggingface
sha256sum omtrackvla_weights.tar > omtrackvla_weights.tar.sha256
```

Keep it as a tar, not as a folder: the Hugging Face cache stores each file as
a symlink into `blobs/`, and cloud-drive folder uploads and Windows
filesystems replace symlinks with copies or drop them. On the target machine:

```bash
sha256sum -c omtrackvla_weights.tar.sha256
mkdir -p /path/to/cache && tar -xf omtrackvla_weights.tar -C /path/to/cache
find /path/to/cache/huggingface/hub -xtype l      # must print nothing (no dangling symlinks)
```

`MODEL_CACHE` in `.env` then points at `/path/to/cache`.

The tarball made on the workstation on 2026-09-10 is on Google Drive:

    https://drive.google.com/drive/folders/1G4hcSg2WfoWTLoY94B0Xxs4UgsgtA9JD

    omtrackvla_weights_20260910.tar          6.0 GB
    omtrackvla_weights_20260910.tar.sha256   2f3ed92009ee6972986cfc7f5881e1961ded4d457604711baa88021feef75ed1

It passed the Phase 0 gate on its own, with the original `cache/` unmounted,
and gave the same waypoints as the original. Download both files through the
browser (or `rclone copy gdrive:<folder>/ .` with a configured remote) and
continue with the `sha256sum -c` line above.

## Hardware the image runs on

- **x86_64 only.** The base is `nvidia/cuda:12.8.1-cudnn-devel-ubuntu24.04`
  with the `cu128` PyTorch wheels. A Jetson (arm64) cannot build or run this
  Dockerfile; it would need an L4T base and a matching torch, which has not
  been attempted.
- **NVIDIA GPU, compute capability 7.0 to 12.0**, NVIDIA driver 570 or newer
  (what CUDA 12.8 requires), `nvidia-container-toolkit`, Docker Compose v2.
  See [A different GPU](#a-different-gpu) for what changes between cards.
- **GPU memory.** The backend alone, measured 2026-09-08 with
  `scripts/measure_backend.py` on the workstation, holds 3.9 GiB as the
  driver sees it with the 31-frame history full, and does not grow with time.
  The whole pipeline observed in `nvidia-smi` during simulator runs sits
  around 4 GiB and peaks a little above 5 GiB. An 8 GB card fits; 6 GB is
  untested.
- **Host RAM and disk.** The compose file asks for `shm_size: 16gb`; that is
  a ceiling on a tmpfs, not an allocation, but lower it on a machine with
  16 GB of RAM. Plan on 80 GB of free disk for the image, its build cache and
  the weights.

## Where the GPU sits

Two layouts work with the code as it is.

**On the robot.** Everything runs in one container on the robot's computer.
Simplest, and the only layout where the latency numbers in this repository
apply.

**Off the robot.** The container runs on a workstation; the robot publishes
its camera and subscribes to `/cmd_vel` over the network. `compose.yaml` uses
`network_mode: host`, so DDS discovery across two hosts needs only the same
`ROS_DOMAIN_ID` on both and multicast between them. Two things then matter:

- The image reaches the node late. `max_image_age` (0.5 s) in the inference
  node and `max_trajectory_age` (0.5 s) in the executor both compare the
  camera stamp with the local clock, so the two hosts must be time-synced
  (chrony). If they cannot be, set `max_trajectory_age` to 0 and rely on
  `trajectory_timeout` alone.
- Raw `sensor_msgs/Image` at 10 Hz is fine on wired Ethernet and marginal on
  Wi-Fi. If the camera driver offers a compressed transport, publish that and
  decompress on the workstation side; the node subscribes to the raw type.

## What the robot must provide

1. **A forward RGB camera** publishing `sensor_msgs/Image` (`rgb8` or
   `bgr8`) with `header.stamp` filled, at 10 Hz or more. Pass its topic as
   `image_topic:=`. Mount it level, facing forward, about 0.6 to 0.75 m above
   the floor, with a horizontal field of view near 90 degrees; that is the
   checkpoint's training camera and what the simulator uses (384x384,
   hfov 90 degrees, 0.63 m). Prefer a square or near-square resolution:
   upstream preprocessing squashes every frame to a square, so a 16:9 sensor
   arrives distorted relative to training.
2. **A base that subscribes to `geometry_msgs/Twist` on `/cmd_vel`.** The
   pipeline publishes `linear.x`, `linear.y` and `angular.z`.
3. Optionally a **laser scan** for the Nav2 collision monitor
   (`enable_collision_monitor:=true`, source topic in
   `collision_monitor.yaml`). Do not enable it without the sensor: it is
   fail-safe and holds the base stopped when no scan arrives.

## Install

On the machine that will run the container:

```bash
git clone <this repository> && cd trackvla_ros2
git submodule update --init --recursive
tar -xf omtrackvla_weights.tar -C /path/to/cache      # weights, see above; or rsync the cache/ dir
cp .env.example .env && $EDITOR .env
docker compose build vla_tracking
docker compose run --rm vla_tracking ./scripts/check_gpu.sh
docker compose run --rm vla_tracking python3 scripts/phase0_check.py
docker compose run --rm vla_tracking ./scripts/build_workspace.sh
docker compose run --rm vla_tracking python3 scripts/measure_backend.py     # VRAM and latency on this GPU
```

`phase0_check.py` loads the checkpoint and runs one forward pass; if it
passes, the model side is done. `measure_backend.py` prints the two numbers
the rest of this document asks you to know: peak GPU memory and the
steady-state inference time on this card.

## A parameter file for the robot

Copy `src/vla_tracking/config/tracking.yaml` to `tracking_<robot>.yaml` and
change only these:

| Parameter | Set it to |
| --- | --- |
| `backend` | `omtrackvla` |
| `image_topic` | the camera topic |
| `max_linear_velocity`, `max_angular_velocity` | the base's real limits. Start at 0.3 m/s and 1.0 rad/s regardless of what the base can do |
| `lateral_policy` | `drop` for any differential-drive base. The simulator keeps `preserve` because Gazebo's DiffDrive silently ignores `linear.y`; a real driver may not. Keep `preserve` only for a holonomic base |
| `trajectory_timeout` | keep 0.3 s unless inference on this GPU takes longer than about 150 ms (see below). This is what stops the base when the model stops producing; the base can coast `trajectory_timeout * max_linear_velocity` metres before it fires |

Leave `linear_scale`, `lateral_scale` and `angular_scale` alone. They convert
the model's normalized output to metres and radians per second and were
derived from the training environment (D019); they describe the checkpoint,
not the robot.

For the velocity smoother, `src/vla_tracking/config/velocity_smoother.yaml`
carries limits derived from the simulated TurtleBot's tip-over threshold.
That derivation does not transfer. Replace `max_velocity`, `min_velocity`,
`max_accel` and `max_decel` with the manufacturer's figures for the base.

## Launch

Use the package launch file, not `sim/launch/tracking_sim.launch.py`, which
wraps it with simulator parameters and `use_sim_time`.

```bash
docker compose up -d vla_tracking
docker exec -it trackvla-ros2-dev /usr/local/bin/entrypoint.sh bash
ros2 launch vla_tracking tracking.launch.py \
    backend:=omtrackvla \
    params_file:=/workspace/trackvla_ros2/src/vla_tracking/config/tracking_<robot>.yaml \
    image_topic:=/your/camera/topic \
    enable_velocity_smoother:=true
```

In a second shell, before sending a goal:

```bash
ros2 topic hz /your/camera/topic     # frames arrive at the expected rate
ros2 topic echo /vla/status          # backend_ready true, task_state idle
ros2 topic echo /cmd_vel             # silent until a goal is active
```

Then:

```bash
ros2 action send_goal /vla/track_target vla_tracking_interfaces/action/TrackTarget \
    "{instruction: 'follow the person'}" --feedback
```

Ctrl-C in the `send_goal` terminal asks the server to cancel; the executor
then stops the base when `trajectory_timeout` elapses. Killing the client
process outright leaves the goal active. See
[running.md](running.md#checking-a-run) for how to tell what is running.

## Bring-up order

Do these in order and do not skip one.

1. Wheels off the floor, `backend:=fake`. Confirms the camera topic, the
   `/cmd_vel` wiring and the smoother chain without the model.
2. Wheels off the floor, `backend:=omtrackvla`. Walk across the camera's
   view and watch `/cmd_vel` follow you.
3. On the floor at 0.3 m/s with one person in view, standing directly ahead
   when the goal is sent.
4. Raise the limits.

## Safety

- A hardware emergency stop is mandatory. `trajectory_timeout` only covers
  the model going silent; it does nothing about the model being wrong.
- Put a `twist_mux` in front of the base with a joystick at higher priority
  than the pipeline's `/cmd_vel`, so an operator can override without
  touching the software.
- `max_*_velocity` scale the whole command down when an axis exceeds its
  limit, preserving the turning radius; they are the only thing between a
  prediction of 1.0 (3.75 m/s after scaling) and the wheels.

## What to expect from the model

The closed-loop and open-loop experiments in
[sim/experiments/textswap/README.md](../sim/experiments/textswap/README.md)
and [sim/experiments/firstframe/README.md](../sim/experiments/firstframe/README.md)
established that the released 0.6B checkpoint follows whichever person it
locks onto first; the instruction text does not choose between people.
Distractor-tracking success in the benchmark is 0.42. On a robot this means:
start with only the target in view, directly ahead, and let others enter
afterwards. Selecting one person out of several by description needs a
detector in front of the tracker or a fine-tuned language pathway; neither
exists here.

## A different GPU

The workstation has an RTX PRO 6000 Blackwell (compute capability 12.0,
96 GB). The next target is an RTX 4070 laptop with 8 GB (Ada, capability
8.9). What changes:

**Kernels.** The torch build's architecture list is
`sm_70 sm_75 sm_80 sm_86 sm_90 sm_100 sm_120`; there is no `sm_89` entry.
That is fine: a CUDA binary built for capability X.y runs on any device X.z
with z >= y, so the 4070 executes the `sm_86` binaries. `check_gpu.sh`
applies that rule and reports which binaries a card without a native entry
will use. Torch's own check only requires the capability to fall inside
7.0 to 12.0.

**Memory.** About 5 GiB peak for the whole pipeline (above). An 8 GB laptop
card fits. If the
laptop drives its display from the NVIDIA GPU rather than the integrated one,
the desktop takes some of the rest; check `nvidia-smi` before launching.

**Speed, and what the node does when a pass takes longer than 100 ms.** The
inference node runs at most `inference_rate` (10 Hz). A pass that overruns
the period does not queue: the loop takes the newest frame and runs again
immediately, so the effective rate becomes 1 / (inference time). The
checkpoint learned a 0.1 s cadence and the executor treats each prediction as
covering that interval, so a slower card stretches the history the model sees
and delays each command; how much that costs in tracking quality has not
been measured. Two consequences are concrete:

- If a pass exceeds `trajectory_timeout` (0.3 s), the executor stops the base
  between predictions and the robot stutters. Raise the timeout to about twice
  the measured inference time; the coasting distance grows with it.
- The 39 to 55 ms in [provenance.md](provenance.md) were measured on the
  workstation with the GPU otherwise idle. The same probe run while five
  evaluation processes shared the GPU gave 120 to 280 ms, so contention alone
  can triple it. Expect a 4070 laptop to be slower than the idle workstation
  figure by a factor that only `measure_backend.py` on that machine can give.

**Laptop specifics.** Run on mains power with the GPU in its performance mode;
on battery, clocks drop and inference time varies from pass to pass, which
shows up as jitter in the command stream. `NVIDIA_DRIVER_CAPABILITIES` can be
reduced to `compute,utility` in `.env` or `compose.yaml` when no Gazebo
rendering is needed. Lower `shm_size` to fit the laptop's RAM.

**Numerics.** A different architecture selects different cuBLAS and attention
kernels, so outputs differ from the workstation's at around 1e-3. The
trajectory is chaotically sensitive to such differences (the text-ablation
runs showed identical inputs diverging under GPU contention), so do not expect
step-for-step agreement between machines. That is not a fault.

**Not for the laptop.** The Gazebo container renders with ogre2 on the GPU
and expects the workstation's setup; running the simulator on the laptop is
out of scope here.
