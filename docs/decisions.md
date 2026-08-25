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
