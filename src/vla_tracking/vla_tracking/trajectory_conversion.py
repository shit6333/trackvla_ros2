"""
Conversion of predicted poses into timed base-velocity segments.

The waypoints a backend predicts are not metres. OmTrackVLA was trained on
normalized base commands, integrated into a path with a fixed constant, so a
waypoint carries "fraction of full speed, times that constant". Dividing by
`dt` therefore recovers the normalized command the model asked for; it is the
exact inverse of the integration that built the training labels, not a
physical distance over time.

The recovered command is not bounded by the network. This checkpoint has no
output activation, and the [-1, 1] range it was trained against belonged to
the simulator, which clipped commands before executing them rather than
teaching the network not to produce them.

Two separate steps follow. `scale_segment` converts to metres and radians per
second using constants that belong to the checkpoint's training environment.
`limit_segment` then brings the result inside what a particular robot can
deliver, scaling the whole command by one factor so the turning radius
survives. Keeping them apart is what lets a prediction above 1.0 be scaled
rather than cut. See D019.

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
    linear_scale: float,
    lateral_scale: float,
    angular_scale: float,
) -> VelocitySegment:
    """
    Turn a normalized segment into metres and radians per second.

    These scales are a property of the checkpoint's training environment, not
    of the robot. They answer "how far did a command of 1.0 move the agent the
    model learned from", and change only when the model does.

    They are deliberately not the robot's limits. Conflating the two forces a
    prediction above 1.0 to be cut rather than scaled, and nothing guarantees
    the model stays below 1.0: this checkpoint has no output activation, and
    the bound it was trained against belonged to the simulator, which clipped
    commands before executing them rather than teaching the network not to
    produce them.
    """
    return VelocitySegment(
        linear_x=segment.linear_x * linear_scale,
        linear_y=segment.linear_y * lateral_scale,
        angular_z=segment.angular_z * angular_scale,
        duration=segment.duration,
    )


def limiting_factor(
    segment: VelocitySegment,
    max_linear: float,
    max_lateral: float,
    max_angular: float,
) -> float:
    """
    Return the factor that brings every axis inside the robot's limits.

    One for a segment already within them, and zero for one carrying a
    non-finite value, which must stop the base rather than propagate.
    """
    for value in (segment.linear_x, segment.linear_y, segment.angular_z):
        if not math.isfinite(value):
            return 0.0

    factor = 1.0
    for value, limit in (
        (segment.linear_x, max_linear),
        (segment.linear_y, max_lateral),
        (segment.angular_z, max_angular),
    ):
        limit = abs(limit)
        if limit > 0.0 and abs(value) > limit:
            factor = min(factor, limit / abs(value))
    return factor


def limit_segment(
    segment: VelocitySegment,
    max_linear: float,
    max_lateral: float,
    max_angular: float,
) -> VelocitySegment:
    """
    Bring a scaled segment inside the robot's limits, preserving its shape.

    The whole command is scaled down by one factor rather than each axis being
    clipped independently. Clipping one axis and not another changes the ratio
    between linear and angular velocity, which is the turning radius, so the
    robot follows an arc the model never asked for. Scaling keeps the arc and
    only slows the traverse.

    The cost is that one saturating axis slows everything, which is the right
    trade for a following task: arriving late on the intended path beats
    arriving on time on a different one.
    """
    factor = limiting_factor(segment, max_linear, max_lateral, max_angular)
    if factor == 0.0:
        # Written out rather than multiplied: nan * 0.0 is nan, so scaling a
        # non-finite command by the zero factor would propagate it to the
        # wheels instead of stopping them.
        return VelocitySegment(
            linear_x=0.0, linear_y=0.0, angular_z=0.0,
            duration=segment.duration,
        )
    return VelocitySegment(
        linear_x=segment.linear_x * factor,
        linear_y=segment.linear_y * factor,
        angular_z=segment.angular_z * factor,
        duration=segment.duration,
    )
