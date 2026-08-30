"""
ROS 2 node that turns predicted trajectories into base velocity commands.

This is the only node that can move the robot, so most of its work is
refusal. Anything malformed, stale, invalid, or unbounded is rejected with a
logged reason, and every path that is not "executing a checked segment right
now" publishes zero velocity.

The model reports a fraction of full speed rather than metres per second, so
`max_linear_velocity` and its siblings are the robot's full-speed values and
are what convert a prediction into a command. They are not a ceiling the
model is trimmed to; the clamp behind the conversion is a separate fuse.

Execution is open loop: each segment is derived from the relative transform
between two consecutive predicted poses and applied for one `dt`. There is no
odometry feedback, so the node does not subscribe to /odom (see D003).
"""

import signal
import threading
import time
from typing import List, Optional

from geometry_msgs.msg import Twist
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from vla_tracking.trajectory_conversion import (
    clamp_segment,
    scale_segment,
    trajectory_to_segments,
    VelocitySegment,
)
from vla_tracking.trajectory_validation import validate_trajectory

from vla_tracking_interfaces.msg import VlaTrajectory

#: Lateral velocity is passed through as the model produced it.
LATERAL_PRESERVE = 'preserve'

#: Lateral velocity is zeroed, for a base that cannot move sideways.
LATERAL_DROP = 'drop'

LATERAL_POLICIES = (LATERAL_PRESERVE, LATERAL_DROP)


class _Plan:
    """The segments derived from one accepted trajectory."""

    def __init__(self, segments: List[VelocitySegment], received_at: float):
        """Record the segments and when they were accepted."""
        self.segments = segments
        self.received_at = received_at
        self.started_at = received_at

    def segment_at(self, elapsed: float) -> Optional[VelocitySegment]:
        """Return the segment covering `elapsed`, or None once exhausted."""
        if elapsed < 0.0:
            return None
        boundary = 0.0
        for segment in self.segments:
            boundary += segment.duration
            if elapsed < boundary:
                return segment
        return None


class TrajectoryExecutorNode(Node):
    """Executes at most a configured number of waypoints per prediction."""

    def __init__(self, **kwargs):
        """Declare parameters and open the trajectory and command interfaces."""
        super().__init__('trajectory_executor_node', **kwargs)

        self.declare_parameter('waypoints_to_execute', 1)
        self.declare_parameter('trajectory_timeout', 0.3)
        self.declare_parameter('command_rate', 20.0)
        self.declare_parameter('max_linear_velocity', 0.2)
        self.declare_parameter('max_lateral_velocity', 0.2)
        self.declare_parameter('max_angular_velocity', 0.5)
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('require_valid', True)
        self.declare_parameter('max_trajectory_age', 0.5)
        self.declare_parameter('lateral_policy', LATERAL_PRESERVE)

        self._waypoints_to_execute = int(
            self.get_parameter('waypoints_to_execute').value
        )
        if self._waypoints_to_execute < 1:
            raise ValueError(
                'waypoints_to_execute must be at least 1, got '
                f'{self._waypoints_to_execute}'
            )

        self._lateral_policy = str(
            self.get_parameter('lateral_policy').value
        )
        if self._lateral_policy not in LATERAL_POLICIES:
            raise ValueError(
                f'lateral_policy must be one of {LATERAL_POLICIES}, got '
                f'{self._lateral_policy!r}'
            )

        self._trajectory_timeout = float(
            self.get_parameter('trajectory_timeout').value
        )
        self._max_linear = float(self.get_parameter('max_linear_velocity').value)
        self._max_lateral = float(
            self.get_parameter('max_lateral_velocity').value
        )
        self._max_angular = float(
            self.get_parameter('max_angular_velocity').value
        )
        self._base_frame = str(self.get_parameter('base_frame').value)
        self._require_valid = bool(self.get_parameter('require_valid').value)
        self._max_trajectory_age = float(
            self.get_parameter('max_trajectory_age').value
        )

        self._plan_lock = threading.Lock()
        self._plan: Optional[_Plan] = None
        self._accepted = 0
        self._rejected = 0
        self._last_reject_reason = ''

        self._command_publisher = self.create_publisher(Twist, 'cmd_vel', 1)
        self._trajectory_subscription = self.create_subscription(
            VlaTrajectory, 'vla/trajectory', self._on_trajectory, 1
        )
        self._command_timer = self.create_timer(
            1.0 / max(float(self.get_parameter('command_rate').value), 1e-3),
            self._publish_command,
        )

        self.get_logger().info(
            f'executing {self._waypoints_to_execute} waypoint(s) per '
            f'prediction, full speed {self._max_linear:.2f} m/s / '
            f'{self._max_angular:.2f} rad/s, lateral policy '
            f'{self._lateral_policy!r}'
        )

    # -- input --------------------------------------------------------------

    def _on_trajectory(self, message: VlaTrajectory) -> None:
        """Validate a trajectory and, if it passes, preempt the current plan."""
        result = validate_trajectory(
            message,
            now_ns=self.get_clock().now().nanoseconds,
            base_frame=self._base_frame,
            require_valid=self._require_valid,
            max_age_seconds=self._max_trajectory_age,
        )
        if not result:
            self._reject(result.reason)
            return

        waypoints = [
            (waypoint.x, waypoint.y, waypoint.theta)
            for waypoint in message.waypoints
        ]
        try:
            segments = trajectory_to_segments(
                waypoints, message.dt, self._waypoints_to_execute
            )
        except ValueError as exc:
            self._reject(f'conversion failed: {exc}')
            return

        if not segments:
            self._reject('no executable segments')
            return

        # A newer prediction always replaces whatever remained of the old one.
        with self._plan_lock:
            self._plan = _Plan(segments, time.monotonic())
            self._accepted += 1

    def _reject(self, reason: str) -> None:
        """Record and log a rejection without disturbing the active plan."""
        with self._plan_lock:
            self._rejected += 1
            self._last_reject_reason = reason
        self.get_logger().warn(
            f'rejected a trajectory: {reason}', throttle_duration_sec=2.0
        )

    # -- output -------------------------------------------------------------

    def _publish_command(self) -> None:
        """Publish the velocity for this instant, or zero when there is none."""
        segment = self._current_segment()
        if segment is None:
            self._command_publisher.publish(Twist())
            return

        # The segment is a fraction of full speed, so scaling is the actual
        # conversion and the clamp behind it is only a fuse.
        limited = clamp_segment(
            scale_segment(
                segment,
                self._max_linear,
                self._max_lateral,
                self._max_angular,
            ),
            self._max_linear,
            self._max_lateral,
            self._max_angular,
        )
        command = Twist()
        command.linear.x = limited.linear_x
        command.linear.y = (
            0.0 if self._lateral_policy == LATERAL_DROP else limited.linear_y
        )
        command.angular.z = limited.angular_z
        self._command_publisher.publish(command)

        if self._lateral_policy == LATERAL_DROP and abs(limited.linear_y) > 1e-3:
            self.get_logger().warn(
                f'discarding {limited.linear_y:+.3f} m/s of commanded lateral '
                f'motion; this base cannot execute it',
                throttle_duration_sec=5.0,
            )

    def _current_segment(self) -> Optional[VelocitySegment]:
        """
        Select the segment to command now, expiring the plan when due.

        Returns None whenever the robot must be still: no plan has arrived,
        the upstream has gone quiet, or every executable segment has been
        applied and no newer prediction has preempted them.
        """
        now = time.monotonic()
        with self._plan_lock:
            plan = self._plan
            if plan is None:
                return None

            if self._trajectory_timeout > 0.0 and \
                    now - plan.received_at > self._trajectory_timeout:
                self._plan = None
                self.get_logger().warn(
                    'no new trajectory within '
                    f'{self._trajectory_timeout:.2f} s; stopping',
                    throttle_duration_sec=2.0,
                )
                return None

            segment = plan.segment_at(now - plan.started_at)
            if segment is None:
                self._plan = None
            return segment

    # -- reporting ----------------------------------------------------------

    @property
    def counters(self):
        """Return accepted and rejected counts and the last rejection reason."""
        with self._plan_lock:
            return self._accepted, self._rejected, self._last_reject_reason

    # -- teardown -----------------------------------------------------------

    def stop(self) -> None:
        """
        Drop the plan and command zero velocity.

        The zero is published more than once with a short gap: a single
        publish issued as the process is tearing down can be lost before the
        middleware sends it, and this is the message that stops the robot.
        """
        with self._plan_lock:
            self._plan = None
        for _ in range(3):
            self._command_publisher.publish(Twist())
            time.sleep(0.02)

    def destroy_node(self) -> bool:
        """
        Command zero velocity before the node disappears.

        This is best effort: a publish issued during teardown may not reach the
        base if the transport is already closing, which is why the executor
        also never latches a non-zero command for longer than one timeout.
        """
        try:
            self.stop()
        except Exception:
            pass
        return super().destroy_node()


def make_termination_handler(node, executor):
    """
    Build the handler that stops the base before the process exits.

    Separated from main so the behaviour can be tested without delivering a
    real signal: what matters is that termination commands zero, not that
    Python routes the signal.
    """
    def _on_terminate(signum, frame):
        node.stop()
        executor.shutdown()

    return _on_terminate


def main(args: Optional[list] = None) -> None:
    """Spin the trajectory executor, stopping the base on the way out."""
    rclpy.init(args=args)
    node = TrajectoryExecutorNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # rclpy installs a SIGINT handler, but SIGTERM terminates a Python process
    # outright without raising KeyboardInterrupt. Without this handler a
    # supervisor or a plain kill would leave the base running at whatever
    # velocity was last commanded.
    previous_sigterm = signal.signal(
        signal.SIGTERM, make_termination_handler(node, executor)
    )
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        signal.signal(signal.SIGTERM, previous_sigterm)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
