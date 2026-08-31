"""
Conversion of predicted poses into timed base-velocity segments.

The waypoints a backend predicts are not metres. OmTrackVLA was trained on
normalized base commands, integrated into a path with a fixed constant, so a
waypoint carries "fraction of full speed, times that constant". Dividing by
`dt` therefore recovers the normalized command the model asked for; it is the
exact inverse of the integration that built the training labels, not a
physical distance over time.

The recovered command is not bounded by the network. Habitat is what bounded
it, clipping to [-1, 1] before scaling each axis by its speed constant, so
`scale_segment` followed by `clamp_segment` reproduces that pair rather than
merely guarding it. See D019.
Turning that into metres per second is a second, separate step that belongs to
the robot rather than to the model: see `scale_segment`.

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
    """
    One constant command to be applied for `duration` seconds.

    Straight out of `trajectory_to_segments` the three components are
    normalized, in [-1, 1]. They only become metres and radians per second
    once `scale_segment` has applied the robot's full-speed values.
    """

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
    Convert consecutive predicted poses into normalized command segments.

    Waypoint 0 is the trajectory origin, so the first segment spans waypoints
    0 to 1. Each segment is the relative transform between two consecutive
    poses, expressed in the frame of the earlier one and divided by `dt`.
    Rotating into the earlier pose's frame matters as soon as the trajectory
    curves: the raw difference is expressed in the prediction frame, but the
    robot executing the segment has already turned by that pose's yaw. That
    rotation is also what makes this the exact algebraic inverse of the
    integration that produced the training labels, so what comes back is the
    command the model asked for rather than an approximation of it.

    The result is normalized, not metric. Pass it through `scale_segment`
    before it reaches a robot.

    One caveat applies only when `max_segments` is large. A backend may
    subsample its waypoints unevenly, in which case some pairs are more than
    one `dt` apart and their segment comes out proportionally too fast. For
    OmTrackVLA the first three segments are evenly spaced and the fourth is
    not, so a value up to 3 is safe and the default of 1 is unaffected.

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


def scale_segment(
    segment: VelocitySegment,
    max_linear: float,
    max_lateral: float,
    max_angular: float,
) -> VelocitySegment:
    """
    Turn a normalized segment into metres and radians per second.

    The model reports a fraction of full speed, so the robot's full-speed
    values are what give that fraction a physical meaning. Multiplying is the
    whole conversion, and it is what keeps the limits out of the model's
    operating range: a command the model actually asked for is reproduced at
    the requested fraction rather than being cut down to a ceiling.

    Each axis carries its own scale because the model normalizes each one
    separately, so they cannot be folded into a single factor.
    """
    return VelocitySegment(
        linear_x=segment.linear_x * max_linear,
        linear_y=segment.linear_y * max_lateral,
        angular_z=segment.angular_z * max_angular,
        duration=segment.duration,
    )


def clamp_segment(
    segment: VelocitySegment,
    max_linear: float,
    max_lateral: float,
    max_angular: float,
) -> VelocitySegment:
    """
    Apply hard actuator limits to an already scaled segment.

    Applied after `scale_segment`, this is a fuse rather than an operating
    condition: the planner bounds its own output, so a scaled command already
    lies within these limits and passes through untouched. What is left for
    this to catch is a backend that escapes that bound, and a non-finite value
    from any source, neither of which may reach the wheels.

    Note that the limit is applied per axis, so a segment that does engage it
    comes out pointing somewhere the model did not ask for. That is acceptable
    for a fuse and would not be acceptable for a routine speed cap.
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
