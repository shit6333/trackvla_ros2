# Design decisions

## D001 — Model-independent ROS boundary

The ROS node publishes a normalized trajectory and uses a backend contract.
OmTrackVLA-specific imports and preprocessing stay in an adapter package. This
allows future tracking and approach models to reuse the ROS interfaces and
executor.

## D002 — Configurable waypoint execution count

There is no separate `first_step` versus `follow_until_update` execution mode.
`waypoints_to_execute` controls the maximum number of future points executed from
one prediction and defaults to one. Every newer prediction preempts the remainder
of the old one.

## D003 — Open-loop first implementation

The initial `trajectory_executor_node` converts predicted pose segments to timed
velocity commands without odometry feedback. It therefore does not subscribe to
`/odom`. Closed-loop tracking is deferred and may later add odometry without
changing the trajectory message.

## D004 — Optional Nav2 safety chain

Nav2 Velocity Smoother and Collision Monitor are installed and configured but
disabled by default. Raw experiments must bypass them completely rather than use
no-op thresholds. Basic message validity, timeout, cancellation, and actuator
hard limits remain enabled in all modes.

## D005 — One CUDA-enabled container initially

ROS interfaces, nodes, and the local OmTrackVLA backend share one Ubuntu 24.04,
ROS 2 Jazzy, Python 3.12, CUDA-enabled container. Separate model serving is only
activated if inference dependencies cannot coexist with Jazzy or later deployment
requires process/host isolation.

## D006 — Package first, simulator second

The project first delivers a buildable and tested ROS package/topic pipeline.
Gazebo and physical-robot integrations are separate later phases and do not shape
the first workspace with premature simulator-specific dependencies.

## D007 — No separate bringup/config package initially

Launch files and configuration live in `vla_tracking`. The initial system is too
small to justify a separate `vla_tracking_bringup` or `vla_tracking_config`
package.

## D008 — Local-first Git workflow

Verified work is committed locally to `main`. The repository has no remote and
is not pushed until the project owner creates the online repository and supplies
its URL. Generated artifacts, model caches, checkpoints, credentials, and local
environment files remain untracked.

## D009 — Version pinning anchored to the OmTrackVLA runtime

Dependency versions are taken from the measured `omtrackvla-dev` container
rather than chosen independently, so that a ROS-side numerical difference can be
attributed to the port and not to a silent dependency upgrade. `torch==2.8.0+cu128`
and `torchvision==0.23.0+cu128` match exactly; CUDA 12.8 is also the minimum
build that ships `sm_120` kernels for the target Blackwell GPU.

Two dependencies deviate because Python 3.12 or ROS 2 Jazzy forces it: numpy
tracks the distro `python3-numpy` 1.26.4 that Jazzy's binary extensions link
against, and pip OpenCV is not installed at all because the inference path
imports no `cv2` and `ros-jazzy-cv-bridge` owns image conversion. Both
deviations are recorded in `docs/environment.md`.

## D010 — Inference stack installed into the system interpreter

Ubuntu 24.04 marks the system interpreter as externally managed (PEP 668).
`rclpy` lives there, so the inference stack is installed beside it with
`PIP_BREAK_SYSTEM_PACKAGES=1`. An isolated virtual environment would hide
`rclpy` from torch or torch from `rclpy`, which defeats the single-container
design in D005.

## D011 — Project-owned `Waypoint2D` instead of `geometry_msgs/Pose2D`

The trajectory waypoint type is defined in `vla_tracking_interfaces` as
`float64 x, y, theta`. `geometry_msgs/Pose2D` has identical fields but ROS has
marked it deprecated since Foxy and states it may be removed in any following
release; building the project's foundational interface on it would put both the
message and every recorded bag at risk. `geometry_msgs/Pose` avoids that but
adds a yaw/quaternion conversion on every publish and every execution step, and
a permanently zero `z`, for a model that only predicts planar poses.

Neither alternative provides native RViz rendering: RViz cannot display a
custom `VlaTrajectory` whatever it contains, so visualization requires a
separate `nav_msgs/Path` debug topic under all three options.

## D012 — The backend owns the temporal history

`vla_inference_node` keeps only the most recent frame; the model's temporal
state lives in the backend and is discarded by `reset()`.

OmTrackVLA's history is not imagery. Each frame is encoded once by the frozen
DINOv3 and SigLIP encoders and pooled to a four-token coarse summary; the
history is 31 of those summaries, about 762 KB in total. Having the node hold
31 raw frames and pass a window on every step would force the encoders to run
31 times per step instead of once.

This supersedes the original Phase 3 wording, which placed a bounded temporal
buffer in the node.

## D013 — Warm-up predictions are published but never executed

Until the history is full the backend left-pads it with its earliest frame,
reproducing the upstream evaluator. Such a prediction is returned with
`warming_up` true and `valid` false: it reaches `/vla/trajectory` and RViz for
visibility, and the executor refuses it, so the base is never driven from a
padded history. `Prediction` rejects the contradictory combination of
`warming_up` and `valid` at construction.

At the measured 25.6 Hz the 31-frame history fills in roughly 1.2 s at full
rate, or 3.1 s if frames are sampled at the checkpoint's 10 Hz cadence.

## D014 — The inference node drops frames instead of queueing them

`vla_inference_node` keeps only the newest camera frame and discards any
earlier unconsumed one. Inference is slower than a camera, so a queue would
accumulate lag until the base were driven from imagery that no longer
describes the world. Dropping is the correct behaviour for a control loop.

Taking a frame removes it, so a stalled camera cannot be silently re-inferred
as though it were live, and `max_image_age` rejects frames that are too old.
