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

from vla_tracking.trajectory_executor_node import (
    make_termination_handler,
    TrajectoryExecutorNode,
)

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
            # Unit scales, so an assertion reads directly as
            # "waypoint difference over dt" without a conversion in the way.
            # The scales are exercised separately in the conversion tests.
            Parameter('linear_scale', value=1.0),
            Parameter('lateral_scale', value=1.0),
            Parameter('angular_scale', value=1.0),
            # Clear of everything under test except test_velocity_limits.
            Parameter('max_linear_velocity', value=1.0),
            Parameter('max_lateral_velocity', value=1.0),
            Parameter('max_angular_velocity', value=2.0),
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

        self._executors = []
        self._threads = []
        for node in (self.node, self.peer):
            # Capped deliberately: the default is one thread per CPU,
            # so two nodes opened 48 for a handful of callbacks and
            # the contention showed up as scheduling stalls long
            # enough to skip an entire trajectory segment.
            executor = MultiThreadedExecutor(
                num_threads=4, context=self.context
            )
            executor.add_node(node)
            self._executors.append(executor)
        self._stop = threading.Event()
        for executor in self._executors:
            thread = threading.Thread(
                target=self._spin, args=(executor,), daemon=True
            )
            thread.start()
            self._threads.append(thread)

    def _spin(self, executor):
        """
        Spin one executor until it is shut down.

        Each node gets its own executor. Sharing one between a node that
        publishes on a timer and a node that observes it starves the observer:
        measured against a 50 Hz publisher, a shared executor delivered 3.5 Hz
        while separate ones delivered the full 50.
        """
        try:
            executor.spin()
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
        for executor in self._executors:
            executor.shutdown()
        for thread in self._threads:
            thread.join(timeout=5.0)
        self.node.destroy_node()
        self.peer.destroy_node()
        rclpy.shutdown(context=self.context)


def _wait_until(predicate, timeout, description):
    """Poll until a predicate holds, failing with a readable message."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.02)
    raise AssertionError(f'timed out after {timeout:.1f}s waiting for {description}')


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
    # 0.01 over dt = 0.1 is a tenth of full speed, and full speed is 0.2 m/s.
    assert moving.linear.x == pytest.approx(0.1, abs=1e-6)
    assert moving.linear.y == pytest.approx(0.0, abs=1e-9)
    assert moving.angular.z == pytest.approx(0.0, abs=1e-9)


#: Steps of 0.01, 0.02 and 0.03, so each segment commands its own speed and
#: which of them were executed can be read straight off the commands.
# Segment durations equal dt, so a longer dt is what keeps a scheduling stall
# from swallowing a whole segment and reporting a waypoint as never executed.
# The steps scale with it, so the three commanded speeds stay 0.02, 0.04 and
# 0.06 m/s.
SLOW_DT = 0.3
UNEQUAL_STEPS = [
    Waypoint2D(x=0.0, y=0.0, theta=0.0),
    Waypoint2D(x=0.03, y=0.0, theta=0.0),
    Waypoint2D(x=0.09, y=0.0, theta=0.0),
    Waypoint2D(x=0.18, y=0.0, theta=0.0),
]

#: Must outlast the three segments so the plan is not timed out mid-way.
SLOW_TIMEOUT = 2.0


def test_only_the_requested_number_of_waypoints_is_executed():
    """
    Check waypoints_to_execute bounds how much of a prediction is followed.

    Two assertions, because neither alone is both direct and robust. The
    segment count is what the parameter actually controls and is checked
    exactly. The commanded speeds are then checked as a subset of what the
    requested waypoints allow, which catches a node that ignored the
    parameter without requiring that every segment be observed: observation
    rate collapses under machine load, and a missed sample must not read as a
    product defect.

    Each configuration runs in its own harness, one after the other. rclpy's
    Context isolates the client library but not the DDS domain, so two
    harnesses alive at once publish and subscribe to each other's topics.
    """
    for requested, allowed in ((1, {0.1}), (3, {0.1, 0.2, 0.3})):
        running = Harness(overrides=[
            Parameter('waypoints_to_execute', value=requested),
            Parameter('trajectory_timeout', value=SLOW_TIMEOUT),
        ])
        try:
            running.wait_for_commands(2)
            baseline = len(running.commands)
            running.publish(waypoints=UNEQUAL_STEPS, dt=SLOW_DT)

            _wait_until(
                lambda: running.node.plan_segment_count > 0,
                5.0,
                'the trajectory to be accepted',
            )
            assert running.node.plan_segment_count == requested, (
                f'asked for {requested} waypoints, the plan holds '
                f'{running.node.plan_segment_count}'
            )

            _wait_until(
                lambda: any(
                    not is_zero(c) for c in running.commands[baseline:]
                ),
                5.0,
                'the base to start moving',
            )
            time.sleep(0.5)

            observed = {
                round(command.linear.x, 6)
                for command in running.commands[baseline:]
                if not is_zero(command)
            }
            assert observed, 'no motion was commanded'
            assert observed <= allowed, (
                f'executing {requested} waypoint(s) commanded {sorted(observed)}, '
                f'which is outside {sorted(allowed)}'
            )
        finally:
            running.shutdown()


def test_silence_upstream_returns_to_zero(harness):
    """
    Check a stalled upstream can never leave a command latched.

    The base must have moved first, so that what stopped it is the timeout
    rather than a plan quietly running out of segments. Only the timeout
    reports why the base stopped, which is the difference between a diagnosed
    stop and a silent one.
    """
    harness.wait_for_commands(2)
    baseline = len(harness.commands)
    harness.publish(
        waypoints=[
            Waypoint2D(x=0.0, y=0.0, theta=0.0),
            Waypoint2D(x=0.02, y=0.0, theta=0.0),
        ]
    )
    # Waiting for a zero to appear rather than sampling the command received
    # at some fixed instant. The harness now keeps up with the publisher, but
    # an assertion on "the last message right now" still depends on delivery
    # landing before the sleep ends, whereas waiting for the state to be
    # reached does not.
    _wait_until(
        lambda: any(not is_zero(c) for c in harness.commands[baseline:]),
        5.0,
        'the base to start moving, without which this proves nothing',
    )
    moved = len(harness.commands)

    _wait_until(
        lambda: any(is_zero(c) for c in harness.commands[moved:]),
        TIMEOUT + 2.0,
        'the timeout to command zero',
    )


def test_a_late_prediction_does_not_interrupt_the_command():
    """
    Check a plan holds rather than stopping between predictions.

    Segment durations come from the constant the backend integrated its
    waypoints with, not from when a replacement will arrive, and inference
    time varies. A plan that stopped when its durations ran out commanded a
    full stop in the interval before the next prediction landed. Here every
    prediction is deliberately later than the segment it replaces.

    A generous trajectory_timeout is used rather than the fixture's. The
    interval between predictions is produced by sleeping in this thread, and
    under load that sleep can overshoot; with a tight timeout the executor
    would then stop for the correct reason and the test would report it as a
    plan expiring early.
    """
    harness = Harness(
        overrides=[Parameter('trajectory_timeout', value=SLOW_TIMEOUT)]
    )
    try:
        harness.wait_for_commands(2)
        baseline = len(harness.commands)
        for _ in range(5):
            harness.publish(
                waypoints=[
                    Waypoint2D(x=0.0, y=0.0, theta=0.0),
                    Waypoint2D(x=0.02, y=0.0, theta=0.0),
                ]
            )
            # dt is 0.1, so each prediction arrives 0.03 s after the segment
            # it replaces ran out, and far inside trajectory_timeout.
            time.sleep(0.13)

        during = harness.commands[baseline:]
    finally:
        harness.shutdown()
    moving = [index for index, c in enumerate(during) if not is_zero(c)]
    assert moving, 'the base never moved'
    stalled = [
        index for index in range(moving[0], moving[-1])
        if is_zero(during[index])
    ]
    assert not stalled, (
        f'{len(stalled)} of {moving[-1] - moving[0]} commands between '
        f'predictions were a full stop; the plan is expiring instead of '
        f'holding'
    )


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
    """
    Check a prediction that escapes its own bound is cut off.

    The planner bounds its output to [-1, 1], so scaling alone keeps a real
    prediction within the limits. This is the fuse behind that: a backend
    emitting 100 times full speed must not reach the wheels.
    """
    harness.wait_for_commands(2)
    harness.publish(
        waypoints=[
            Waypoint2D(x=0.0, y=0.0, theta=0.0),
            # 10 over dt = 0.1 is 100x full speed.
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
    assert moving.linear.x == pytest.approx(1.0)


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


class _RecordingExecutor:
    """Stands in for the ROS executor so no real spin loop is involved."""

    def __init__(self):
        """Start with no shutdown recorded."""
        self.shutdown_called = False

    def shutdown(self):
        """Record that termination asked the executor to stop."""
        self.shutdown_called = True


def test_termination_stops_the_base(harness):
    """
    Check that terminating the process commands zero.

    SIGTERM ends a Python process without raising KeyboardInterrupt, so the
    teardown in main's finally block never runs on that path. Without an
    explicit handler the base would keep moving at whatever velocity was last
    commanded, which is the worst failure this node can have.
    """
    harness.wait_for_commands(2)
    harness.publish()

    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        if harness.commands and not is_zero(harness.commands[-1]):
            break
        time.sleep(0.01)
    assert not is_zero(harness.commands[-1]), 'the base was never moving'

    fake_executor = _RecordingExecutor()
    handler = make_termination_handler(harness.node, fake_executor)
    handler(15, None)

    assert fake_executor.shutdown_called, 'termination did not stop the spin'
    assert is_zero(harness.commands[-1]), \
        'termination left a non-zero command as the last thing published'
