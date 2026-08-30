# System architecture

## Scope of the first implementation

The first implementation packages the model-facing ROS pipeline only. It does
not yet include Gazebo worlds, a robot description, a real-robot driver, SLAM,
localization, or a Nav2 Controller Server.

The minimum runtime data flow is:

```text
RGB camera + text instruction
              |
              v
      vla_inference_node
              |
              | /vla/trajectory
              v
   trajectory_executor_node
              |
              | /cmd_vel
              v
       external robot base
```

The camera and robot base are external dependencies. During package-only
development they may be replaced by test publishers and subscribers.

## Required nodes

### `vla_inference_node`

Responsibilities:

- accept and cancel a target-tracking task;
- subscribe to monocular RGB images;
- maintain the temporal image history required by the selected backend;
- load and reset a model backend;
- schedule inference without blocking image ingestion;
- publish the complete predicted trajectory, status, and optional debug image;
- publish no robot velocity commands.

Inputs:

| Interface | Type | Meaning |
| --- | --- | --- |
| `/vla/track_target` | `vla_tracking_interfaces/action/TrackTarget` | Start or cancel a text-conditioned tracking task. |
| `/camera/image_raw` | `sensor_msgs/msg/Image` | Monocular RGB observation. The actual camera topic is remappable. |

Outputs:

| Topic | Type | Meaning |
| --- | --- | --- |
| `/vla/trajectory` | `vla_tracking_interfaces/msg/VlaTrajectory` | Short-horizon poses relative to the robot at prediction time. |
| `/vla/status` | `vla_tracking_interfaces/msg/VlaStatus` | Task state, backend state, inference timing, and errors. |
| `/vla/debug_image` | `sensor_msgs/msg/Image` | Optional visualization overlay. |

Proposed task states are `IDLE`, `WARMING_UP`, `TRACKING`, `ERROR`, and
`STOPPED`. Receiving a new task or cancelling the current task clears temporal
model state and invalidates the previous trajectory.

### `trajectory_executor_node`

Responsibilities:

- subscribe to model trajectories;
- reject stale, malformed, non-finite, or incorrectly framed trajectories;
- convert successive relative poses into timed base-velocity segments;
- execute at most `waypoints_to_execute` segments from a prediction;
- immediately preempt the remaining old segments when a new prediction arrives;
- apply hard robot velocity limits;
- publish a zero velocity after execution, timeout, cancellation, or error.

Input:

| Topic | Type |
| --- | --- |
| `/vla/trajectory` | `vla_tracking_interfaces/msg/VlaTrajectory` |

Output in the default configuration:

| Topic | Type |
| --- | --- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` |

The first implementation is an open-loop trajectory executor, not an
odometry-feedback trajectory controller. It therefore does not require `/odom`.
A future closed-loop implementation may add odometry and TF without changing the
model-facing trajectory interface.

## Trajectory semantics

The current OmTrackVLA planner predicts eight poses shaped as `[x, y, theta]`.
The released evaluator predicts a complete trajectory on every environment step,
selects `tau[0, 1]`, converts it with `dt = 0.1`, executes one environment step,
then observes and infers again. The remaining predicted points are used for
future supervision and visualization rather than queued for mandatory execution.

The ROS representation must make previously implicit values explicit:

```text
std_msgs/Header header       # observation stamp and prediction frame
string backend_name
Waypoint2D[] waypoints       # project-owned; float64 x, y, theta
float32 dt                   # seconds between trajectory poses
bool valid
string status
```

`Waypoint2D` is defined in `vla_tracking_interfaces` rather than reused from
`geometry_msgs`. `geometry_msgs/Pose2D` carries the same three fields but ROS
has marked it deprecated since Foxy and may remove it, which would invalidate
both the interface and every recorded bag. `geometry_msgs/Pose` would instead
force a yaw/quaternion conversion on every publish and every execution step for
a model that only predicts planar poses, and would stop the message from
expressing that the trajectory is planar. Neither alternative buys native RViz
rendering, since RViz cannot display a custom message regardless of its
contents; visualization needs a separate `nav_msgs/Path` debug topic either
way. See decision D011.

Initial conventions, to be confirmed with checkpoint-level tests:

- `header.frame_id` is `base_link`;
- waypoint zero represents the current pose or trajectory origin;
- executable points begin at index one;
- `x` is forward, `y` is lateral, and `theta` is yaw;
- `x`, `y` and `theta` are normalized, in [-1, 1] once divided by `dt`, and
  become metres and radians per second only after the executor applies the
  robot's full-speed values;
- `dt` comes from backend/checkpoint metadata and is never inferred by the
  executor.

### Configurable execution horizon

The executor parameter is:

```yaml
waypoints_to_execute: 1
```

For a prediction `A`:

- value `1`: execute only the segment ending at `A[1]`, then stop unless a new
  prediction arrives;
- value `3`: execute segments ending at `A[1]`, `A[2]`, and `A[3]` while waiting
  for a newer prediction;
- a newer prediction always preempts all unexecuted points in `A`;
- reaching the configured count or end of trajectory produces zero velocity;
- valid values are positive and are clamped or rejected when larger than the
  number of executable points.

Executing multiple segments remains open-loop in the first implementation. Each
segment is recovered from the relative transform between consecutive predicted
poses and is applied for one `dt`. Closed-loop path tracking is separate future
work.

Source reference: the current upstream behavior is implemented in
[`trained_agent.py`](https://github.com/om-ai-lab/OmTrackVLA/blob/e9cb1fbd57f8dbcf98c460914251cbcbe20f2f57/trained_agent.py#L284-L312)
and its planner action conversion.

## Replaceable model backends

`vla_inference_node` depends on a project-owned backend contract, not directly on
OmTrackVLA classes:

```python
class VLABackend(Protocol):
    @property
    def name(self) -> str: ...
    def configure(self, config: Mapping[str, Any]) -> None: ...
    def reset(self, instruction: str) -> None: ...
    def infer(self, observation: Observation) -> Prediction: ...
    def shutdown(self) -> None: ...
```

The normalized types are:

```text
Observation                 Prediction
  rgb: HxWx3 uint8            waypoints: [N, 3] normalized, see below
  stamp_ns: int               dt: seconds
  instruction: str            frame_id: str
                              stamp_ns: int, copied from the observation
                              valid: bool
                              warming_up: bool
                              status: str
```

Timestamps are integer nanoseconds rather than ROS message types so that a
backend and its conformance tests can run without a sourced overlay.

The backend owns whatever temporal state its model needs, and `reset()`
discards it. The node keeps only the most recent frame. See D012.

Backends are discovered through the `vla_tracking.backends` entry point group
rather than imported by name, which is what keeps `vla_tracking` free of any
import dependency on an adapter. A shared conformance suite in
`vla_tracking.backend_conformance` is run against every backend, so "the
backends satisfy one contract" is verified rather than asserted.

The first adapter is `OmTrackVLABackend`. Future fine-tuned tracking or approach
models can ship sibling adapter packages while keeping the same action, topics,
trajectory executor, and optional safety chain.

## Optional Nav2 command processing

The optional chain is:

```text
trajectory_executor_node
        | /cmd_vel_raw
        v
nav2_velocity_smoother             disabled by default
        | /cmd_vel_smoothed
        v
nav2_collision_monitor             disabled by default
        | /cmd_vel
        v
robot base
```

Launch arguments:

```text
enable_velocity_smoother:=false
enable_collision_monitor:=false
```

When both are disabled, launch remapping connects executor output directly to
`/cmd_vel`. When enabled, exactly one final node publishes `/cmd_vel`.

The Collision Monitor may later consume `/scan` or a depth-derived point cloud.
These sensors are not model inputs and are not required by the first package-only
implementation.

Regardless of optional Nav2 state, format checks, finite-value checks, trajectory
timeout, task cancellation, and hard actuator limits remain enabled. These guard
software integrity and do not constitute learned obstacle avoidance.
