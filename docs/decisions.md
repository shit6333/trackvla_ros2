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

## D015 — Lateral velocity is passed through by default

The model predicts `[x, y, theta]`, and the upstream evaluator executes all
three against a holonomic simulator agent. The executor therefore publishes
`linear.y` as computed by default (`lateral_policy: preserve`), which is
lossless and matches the behaviour the checkpoint was evaluated under.

`lateral_policy: drop` zeroes it for a base that cannot move sideways, and
logs how much lateral motion was discarded rather than silently swallowing it.

Folding lateral offset into steering is deliberately not implemented. It would
be a control law, not a conversion, and the first executor is open loop by
D003. The choice belongs with the target robot, which is not yet selected;
until then neither default can be validated against hardware.

## D016 — Staleness is checked two independent ways

`trajectory_timeout` measures time since the executor last accepted a
trajectory and needs no clock agreement with the camera. It is the guard that
stops the base when the upstream dies.

`max_trajectory_age` compares the trajectory's observation stamp with the
executor's clock, and catches the case the timeout cannot see: an upstream
that keeps publishing but is recycling a frozen frame. It defaults to 0.5 s.
An unsynchronised camera clock makes it reject everything, which halts the
robot and logs the measured age, so the failure is safe and diagnosable rather
than silent.

## D017 — Command-chain wiring is computed by one pure function

`vla_tracking.cmd_vel_routing.plan_cmd_vel_chain` decides every topic name in
the optional Nav2 chain, and the launch file consumes its result rather than
scattering conditional remappings through a launch description.

The failure this prevents is specific: if two nodes end up publishing the
final command topic, the base acts on whichever message arrives last and its
behaviour becomes non-deterministic. Concentrating the decision in a pure
function makes the invariant testable exhaustively; all four combinations are
checked in unit tests and were also verified live by counting publishers on
`/cmd_vel`.

The collision monitor is always last in the chain. It must judge the command
that will actually be sent, so smoothing has to happen before it, never after.

## D018 — Nodes stop the base on SIGTERM, not only on SIGINT

`rclpy` installs a SIGINT handler, but SIGTERM ends a Python process outright
without raising `KeyboardInterrupt`, so a `finally` block never runs on that
path. A supervisor, a container stop, or a plain `kill` would therefore have
left the base moving at whatever velocity was last commanded.

Both nodes now install a SIGTERM handler. The executor's commands zero before
exiting, and publishes it more than once with a short gap, because a single
publish issued during teardown can be lost before the middleware sends it.

This is a best-effort guarantee. `SIGKILL` defeats it, so it does not replace
either the executor's own `trajectory_timeout` or a watchdog on the base.

## D019 — Velocity limits scale the model's output, they do not cap it

OmTrackVLA predicts a fraction of full speed, not metres per second. It was
trained on Habitat base commands, which that simulator clips to `[-1, 1]` and
multiplies by a per-axis speed constant, and its training labels integrate
those commands with a bookkeeping constant of `0.1`. Dividing a waypoint by
`dt` inverts that integration and returns the command; the planner's output
`tanh` bounds the result to `[-1, 1]`.

The executor therefore multiplies by `max_linear_velocity` and its siblings
rather than clamping to them. Those parameters are the robot's full speed per
axis, and each axis needs its own because the model normalizes each one
separately. No metric scale can be inherited from the simulator, whose
configured maxima are not physically calibrated.

Treating the output as metric had a specific consequence: a prediction of
`0.49` was read as `0.49 m/s` against a `0.2` limit, so the clamp engaged on
every step and the executor became a bang-bang controller with a constant
forward speed. Because the clamp is applied per axis it also changed the
commanded direction, which scaling does not.

The clamp is retained behind the conversion as a fuse. A prediction the
planner bounded passes through it untouched, so it now catches only a backend
that escapes its own bound and non-finite values from any source.
