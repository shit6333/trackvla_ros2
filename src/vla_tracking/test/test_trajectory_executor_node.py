"""
End-to-end tests for trajectory_executor_node over real topics.

This is the only node that can move the robot, so the tests are written around
what must never happen: motion without a valid trajectory, a latched command
after the upstream goes quiet, and any unbounded or non-finite value reaching
/cmd_vel.
"""

import threading
import time

from geometry_msgs.msg import Twist
import pytest
import rclpy
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter

from vla_tracking.trajectory_executor_node import TrajectoryExecutorNode

from vla_tracking_interfaces.msg import VlaTrajectory, Waypoint2D

COMMAND_RATE = 50.0
TRAJECTORY_DT = 0.1
TIMEOUT = 0.4


def is_zero(command: Twist) -> bool:
    """Report whether a command would leave the base still."""
    return (
        command.linear.x == 0.0
        and command.linear.y == 0.0
        and command.angular.z == 0.0
    )


class Harness:
    """Runs the executor alongside a publisher and a command recorder."""

    def __init__(self, overrides=None):
        """Start the executor and a peer node on a background executor."""
        self.context = Context()
        rclpy.init(context=self.context)

        parameters = [
            Parameter('command_rate', value=COMMAND_RATE),
            Parameter('trajectory_timeout', value=TIMEOUT),
            Parameter('max_linear_velocity', value=0.2),
            Parameter('max_lateral_velocity', value=0.2),
            Parameter('max_angular_velocity', value=0.5),
            # The staleness check is exercised in the validation unit tests;
            # here it would only couple every case to wall-clock scheduling.
            Parameter('max_trajectory_age', value=0.0),
        ]
        parameters.extend(overrides or [])

        self.node = TrajectoryExecutorNode(
            context=self.context, parameter_overrides=parameters
        )
        self.peer = Node('test_peer', context=self.context)

        self.commands = []
        self.peer.create_subscription(
            Twist, 'cmd_vel', lambda msg: self.commands.append(msg), 10
        )
        self.publisher = self.peer.create_publisher(
            VlaTrajectory, 'vla/trajectory', 1
        )

        self.executor = MultiThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.executor.add_node(self.peer)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()

    def _spin(self):
        """
        Spin on the executor's own thread pool.

        A hand-rolled spin_once loop services every callback from one thread,
        which lets subscription queues drain far behind real time and makes
        any timing assertion measure the harness rather than the node.
        """
        try:
            self.executor.spin()
        except Exception:
            pass

    def publish(self, **kwargs):
        """Publish a trajectory built from the given overrides."""
        self.publisher.publish(make_trajectory(**kwargs))

    def wait_for_commands(self, count, timeout=5.0):
        """Block until at least `count` commands have been recorded."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if len(self.commands) >= count:
                return
            time.sleep(0.01)
        raise AssertionError(
            f'only {len(self.commands)} commands arrived, wanted {count}'
        )

    def shutdown(self):
        """Tear the harness down."""
        self._stop.set()
        self.executor.shutdown()
        self._thread.join(timeout=5.0)
        self.node.destroy_node()
        self.peer.destroy_node()
        rclpy.shutdown(context=self.context)


def make_trajectory(
    *, valid=True, dt=TRAJECTORY_DT, frame_id='base_link', waypoints=None
):
    """Build a trajectory that is executable unless told otherwise."""
    message = VlaTrajectory()
    message.header.frame_id = frame_id
    message.backend_name = 'fake'
    message.dt = dt
    message.valid = valid
    message.waypoints = waypoints if waypoints is not None else [
        Waypoint2D(x=0.0, y=0.0, theta=0.0),
        Waypoint2D(x=0.01, y=0.0, theta=0.0),
        Waypoint2D(x=0.02, y=0.0, theta=0.0),
        Waypoint2D(x=0.03, y=0.0, theta=0.0),
    ]
    return message


@pytest.fixture
def harness():
    """Provide a running harness with guaranteed teardown."""
    running = Harness()
    try:
        yield running
    finally:
        running.shutdown()


def test_idle_output_is_zero(harness):
    """Check the base is commanded still until something valid arrives."""
    harness.wait_for_commands(5)
    assert all(is_zero(command) for command in harness.commands)


def test_a_valid_trajectory_produces_the_expected_velocity(harness):
    """Check the first executable waypoint becomes the commanded speed."""
    harness.wait_for_commands(2)
    harness.publish()

    deadline = time.monotonic() + 2.0
    moving = None
    while time.monotonic() < deadline:
        if harness.commands and not is_zero(harness.commands[-1]):
            moving = harness.commands[-1]
            break
        time.sleep(0.01)

    assert moving is not None, 'the executor never commanded motion'
    # 0.01 m over dt = 0.1 s.
    assert moving.linear.x == pytest.approx(0.1, abs=1e-6)
    assert moving.linear.y == pytest.approx(0.0, abs=1e-9)
    assert moving.angular.z == pytest.approx(0.0, abs=1e-9)


def _count_moving_commands(running, settle=0.6):
    """Publish one trajectory and count the commands that move the base."""
    running.wait_for_commands(2)
    baseline = len(running.commands)
    running.publish()
    time.sleep(settle)
    return sum(
        0 if is_zero(command) else 1
        for command in running.commands[baseline:]
    )


def test_only_the_requested_number_of_waypoints_is_executed(harness):
    """
    Check waypoints_to_execute bounds how long one prediction drives the base.

    The trajectory carries three executable waypoints. Counting the commands
    that move the base is measured against a node configured to execute three
    of them, so the assertion compares two runs of the same code rather than
    depending on an absolute delivery rate.
    """
    one_segment = _count_moving_commands(harness)
    assert one_segment > 0, 'the single-waypoint run never moved'

    three = Harness(
        overrides=[Parameter('waypoints_to_execute', value=3)]
    )
    try:
        three_segments = _count_moving_commands(three)
    finally:
        three.shutdown()

    assert three_segments > one_segment * 2, (
        f'executing three waypoints produced {three_segments} moving commands '
        f'against {one_segment} for one; waypoints_to_execute is not bounding '
        f'execution'
    )


def test_silence_upstream_returns_to_zero(harness):
    """Check a stalled upstream can never leave a command latched."""
    harness.wait_for_commands(2)
    harness.publish(
        waypoints=[
            Waypoint2D(x=0.0, y=0.0, theta=0.0),
            Waypoint2D(x=0.02, y=0.0, theta=0.0),
        ]
    )
    time.sleep(TIMEOUT + 0.3)
    assert is_zero(harness.commands[-1]), \
        'a command was still latched after the timeout'


def test_an_invalid_trajectory_never_moves_the_base(harness):
    """Check a warm-up or failed prediction produces no motion at all."""
    harness.wait_for_commands(2)
    baseline = len(harness.commands)
    harness.publish(valid=False)
    time.sleep(0.3)
    assert all(
        is_zero(command) for command in harness.commands[baseline:]
    ), 'an invalid trajectory produced motion'


def test_non_finite_waypoints_never_reach_cmd_vel(harness):
    """Check NaN is refused rather than clamped after the fact."""
    harness.wait_for_commands(2)
    baseline = len(harness.commands)
    harness.publish(
        waypoints=[
            Waypoint2D(x=0.0, y=0.0, theta=0.0),
            Waypoint2D(x=float('nan'), y=0.0, theta=0.0),
        ]
    )
    time.sleep(0.3)
    for command in harness.commands[baseline:]:
        assert is_zero(command)


def test_velocity_limits_are_enforced(harness):
    """Check an absurd prediction is bounded rather than passed through."""
    harness.wait_for_commands(2)
    harness.publish(
        waypoints=[
            Waypoint2D(x=0.0, y=0.0, theta=0.0),
            # 10 m in 0.1 s would be 100 m/s.
            Waypoint2D(x=10.0, y=0.0, theta=0.0),
        ]
    )

    deadline = time.monotonic() + 2.0
    moving = None
    while time.monotonic() < deadline:
        if harness.commands and not is_zero(harness.commands[-1]):
            moving = harness.commands[-1]
            break
        time.sleep(0.01)

    assert moving is not None
    assert moving.linear.x == pytest.approx(0.2)


def test_a_newer_trajectory_preempts_the_previous_one(harness):
    """Check the remainder of an older prediction is abandoned at once."""
    harness.wait_for_commands(2)
    harness.publish(
        waypoints=[
            Waypoint2D(x=0.0, y=0.0, theta=0.0),
            Waypoint2D(x=0.01, y=0.0, theta=0.0),
        ]
    )
    time.sleep(0.02)
    harness.publish(
        waypoints=[
            Waypoint2D(x=0.0, y=0.0, theta=0.0),
            Waypoint2D(x=0.015, y=0.0, theta=0.0),
        ]
    )

    deadline = time.monotonic() + 2.0
    observed = None
    while time.monotonic() < deadline:
        if harness.commands and not is_zero(harness.commands[-1]):
            observed = harness.commands[-1].linear.x
            if observed == pytest.approx(0.15, abs=1e-6):
                break
        time.sleep(0.01)

    assert observed == pytest.approx(0.15, abs=1e-6), \
        f'expected the newer prediction to take over, saw {observed}'


def test_stop_commands_zero_immediately(harness):
    """Check the explicit stop path halts the base."""
    harness.wait_for_commands(2)
    harness.publish()
    time.sleep(0.02)
    harness.node.stop()
    harness.wait_for_commands(len(harness.commands) + 2)
    assert is_zero(harness.commands[-1])
