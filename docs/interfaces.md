# Interfaces

Reference for the topics, action, and parameters the pipeline exposes. Names
below are the defaults; all of them are remappable.

## Data flow

```text
/camera/image_raw ──► vla_inference_node ──► /vla/trajectory ──► trajectory_executor_node ──► /cmd_vel
                              │
                              └──────────────► /vla/status
        /vla/track_target (action)
```

## Messages

### `vla_tracking_interfaces/msg/Waypoint2D`

| Field | Type | Meaning |
| --- | --- | --- |
| `x` | float64 | forward, normalized |
| `y` | float64 | lateral, left positive, normalized |
| `theta` | float64 | yaw, normalized |

Normalized, not metric: divided by `dt` these give the fraction of full speed
the model asked for, bounded to [-1, 1]. The executor multiplies by the
robot's `max_*_velocity` to obtain metres and radians per second.

Project-owned rather than `geometry_msgs/Pose2D`, which ROS deprecated in
Foxy. See D011.

### `vla_tracking_interfaces/msg/VlaTrajectory`

| Field | Type | Meaning |
| --- | --- | --- |
| `header.stamp` | Time | **observation** time, not publication time |
| `header.frame_id` | string | frame the waypoints are relative to |
| `backend_name` | string | which model produced this |
| `waypoints` | Waypoint2D[] | index 0 is the origin; executable points start at 1 |
| `dt` | float32 | the constant the waypoints were integrated with; divide by it to recover the command. Not a duration |
| `valid` | bool | false means it must not be executed |
| `status` | string | detail, especially when invalid |

`valid` defaults to false, so an unpopulated message can never be executed.

### `vla_tracking_interfaces/msg/VlaStatus`

Task state constants: `IDLE=0`, `WARMING_UP=1`, `TRACKING=2`, `ERROR=3`,
`STOPPED=4`.

| Field | Type | Meaning |
| --- | --- | --- |
| `task_state` | uint8 | one of the constants above |
| `instruction` | string | active instruction, empty when idle |
| `backend_name` | string | configured backend |
| `backend_ready` | bool | whether weights loaded |
| `last_inference_duration` | float32 | seconds spent in the last inference |
| `last_observation_stamp` | Time | observation behind the last prediction |
| `predictions_published` | uint32 | count for the current task |
| `error_message` | string | populated only in `ERROR` |

## Action

### `/vla/track_target` — `vla_tracking_interfaces/action/TrackTarget`

| Section | Field | Meaning |
| --- | --- | --- |
| Goal | `instruction` | natural-language target description |
| Result | `final_status` | `VlaStatus` at the moment the task ended |
| Result | `predictions_published` | total for that task |
| Result | `message` | why it ended |
| Feedback | `status` | the same `VlaStatus` published periodically |

A new goal preempts the active one and resets the backend's temporal state. An
empty instruction is rejected.

```bash
ros2 action send_goal /vla/track_target \
    vla_tracking_interfaces/action/TrackTarget \
    "{instruction: 'follow the person in the red shirt'}" --feedback
```

## Parameters

### `vla_inference_node`

| Parameter | Default | Meaning |
| --- | --- | --- |
| `backend` | `fake` | entry point name; `fake` or `omtrackvla` |
| `image_topic` | `/camera/image_raw` | monocular RGB input |
| `base_frame` | `base_link` | frame stamped on predictions |
| `inference_rate` | `10.0` | Hz; matches the checkpoint's training cadence |
| `max_image_age` | `0.5` | seconds; 0 disables |
| `status_rate` | `5.0` | Hz |
| `model_dir` | `''` | falls back to `HF_MODEL_DIR` |
| `source_path` | `''` | falls back to `OMTRACKVLA_SRC` |
| `require_cuda` | `true` | refuse to run inference on the CPU |
| `history_length` | `0` | 0 takes the value the checkpoint records |

### `trajectory_executor_node`

| Parameter | Default | Meaning |
| --- | --- | --- |
| `waypoints_to_execute` | `1` | segments driven per prediction |
| `trajectory_timeout` | `0.3` | seconds without an accepted trajectory before stopping; the only thing that ends a plan nothing has preempted |
| `command_rate` | `20.0` | Hz |
| `max_linear_velocity` | `0.2` | **see the warning below** |
| `max_lateral_velocity` | `0.2` | m/s |
| `max_angular_velocity` | `0.5` | rad/s |
| `base_frame` | `base_link` | required `frame_id`; empty disables the check |
| `require_valid` | `true` | refuse trajectories the backend marked invalid |
| `max_trajectory_age` | `0.5` | seconds; 0 disables, needs a synchronised camera clock |
| `lateral_policy` | `preserve` | `preserve` or `drop` |

> The three `max_*_velocity` parameters are the robot's full speed per axis
> and are the conversion factors, not a ceiling: a prediction of `0.49`
> forward commands `0.49 * max_linear_velocity`. Each axis carries its own
> value because the model normalizes each one separately. The clamp behind the
> conversion is a fuse for a backend that escapes its own bound.
>
> **The defaults are not validated against any robot.** Set them from the
> target platform's real capability before any experiment whose result depends
> on speed, because they decide how fast the robot moves.

## Launch arguments

| Argument | Default | Meaning |
| --- | --- | --- |
| `params_file` | `config/tracking.yaml` | parameters for both nodes |
| `backend` | `fake` | overrides the parameter file |
| `image_topic` | `/camera/image_raw` | overrides the parameter file |
| `enable_velocity_smoother` | `false` | insert the Nav2 smoother |
| `enable_collision_monitor` | `false` | insert the Nav2 collision monitor |

The last enabled stage publishes `/cmd_vel`; with both disabled the executor
publishes it directly. Exactly one publisher owns the topic in every
combination (D017).

| smoother | monitor | executor publishes | `/cmd_vel` owner |
| --- | --- | --- | --- |
| off | off | `cmd_vel` | `trajectory_executor_node` |
| on | off | `cmd_vel_raw` | `velocity_smoother` |
| off | on | `cmd_vel_raw` | `collision_monitor` |
| on | on | `cmd_vel_raw` | `collision_monitor` |
