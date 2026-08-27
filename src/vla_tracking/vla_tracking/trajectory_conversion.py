"""
Conversion of predicted poses into timed base-velocity segments.

Pure geometry with no ROS dependency, so the arithmetic that ultimately turns
wheels can be tested exhaustively without a running graph.
"""

from dataclasses import dataclass
import math
from typing import List, Sequence, Tuple

#: A waypoint as forward, lateral, yaw.
Waypoint = Tuple[float, float, float]


@dataclass(frozen=True)
class VelocitySegment:
    """One constant-velocity command to be applied for `duration` seconds."""

    linear_x: float
    linear_y: float
    angular_z: float
    duration: float


def wrap_angle(angle: float) -> float:
    """Fold an angle into [-pi, pi] so a turn never takes the long way round."""
    return math.atan2(math.sin(angle), math.cos(angle))


def trajectory_to_segments(
    waypoints: Sequence[Waypoint],
    dt: float,
    max_segments: int,
) -> List[VelocitySegment]:
    """
    Convert consecutive predicted poses into velocity segments.

    Waypoint 0 is the trajectory origin, so the first segment spans waypoints
    0 to 1. Each segment is the relative transform between two consecutive
    poses, expressed in the frame of the earlier one and divided by `dt`.
    Rotating into the earlier pose's frame matters as soon as the trajectory
    curves: the raw difference is expressed in the prediction frame, but the
    robot executing the segment has already turned by that pose's yaw.

    :param max_segments: Upper bound on how many segments are produced. Fewer
        are returned when the trajectory is short.
    """
    if dt <= 0.0 or not math.isfinite(dt):
        raise ValueError(f'dt must be finite and positive, got {dt}')
    if max_segments < 1:
        raise ValueError(f'max_segments must be at least 1, got {max_segments}')

    usable = min(max_segments, len(waypoints) - 1)
    segments = []
    for index in range(max(usable, 0)):
        x0, y0, yaw0 = waypoints[index]
        x1, y1, yaw1 = waypoints[index + 1]

        delta_x = x1 - x0
        delta_y = y1 - y0
        cos0 = math.cos(yaw0)
        sin0 = math.sin(yaw0)

        forward = cos0 * delta_x + sin0 * delta_y
        lateral = -sin0 * delta_x + cos0 * delta_y
        turn = wrap_angle(yaw1 - yaw0)

        segments.append(
            VelocitySegment(
                linear_x=forward / dt,
                linear_y=lateral / dt,
                angular_z=turn / dt,
                duration=dt,
            )
        )
    return segments


def clamp_segment(
    segment: VelocitySegment,
    max_linear: float,
    max_lateral: float,
    max_angular: float,
) -> VelocitySegment:
    """
    Apply hard actuator limits to a segment.

    Clamping distorts the geometry the model asked for, and that is the point:
    the model is a black box that may emit an arbitrary number, and no such
    number may reach the wheels unbounded. A clamped segment no longer tracks
    the predicted path exactly.
    """
    return VelocitySegment(
        linear_x=_clamp(segment.linear_x, max_linear),
        linear_y=_clamp(segment.linear_y, max_lateral),
        angular_z=_clamp(segment.angular_z, max_angular),
        duration=segment.duration,
    )


def _clamp(value: float, limit: float) -> float:
    """Bound a value to +/- limit, mapping a non-finite value to zero."""
    if not math.isfinite(value):
        return 0.0
    return max(-abs(limit), min(abs(limit), value))
