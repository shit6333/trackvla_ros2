"""
End-to-end tests for vla_inference_node over the real ROS action and topics.

These run against the fake backend, so they need no GPU and no weights, and
they exercise the paths that only appear once messages actually flow: goal
acceptance, warm-up gating, preemption by a new instruction, and cancellation.
"""

import threading
import time

from cv_bridge import CvBridge
import numpy as np
import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from vla_tracking.inference_node import VlaInferenceNode

from vla_tracking_interfaces.action import TrackTarget
from vla_tracking_interfaces.msg import VlaStatus, VlaTrajectory

IMAGE_TOPIC = 'test/image'
HISTORY_LENGTH = 3
INSTRUCTION = 'follow the person in the red shirt'
OTHER_INSTRUCTION = 'follow the person in the blue coat'


class Harness:
    """Runs the node under test alongside a camera and a client."""

    def __init__(self):
        """Start both nodes on a background multi-threaded executor."""
        self.context = Context()
        rclpy.init(context=self.context)

        self.node = VlaInferenceNode(
            context=self.context,
            parameter_overrides=[
                Parameter('backend', value='fake'),
                Parameter('image_topic', value=IMAGE_TOPIC),
                Parameter('history_length', value=HISTORY_LENGTH),
                Parameter('inference_rate', value=50.0),
                # Synthetic frames carry a clock stamp; ageing them out would
                # make the test depend on scheduling latency.
                Parameter('max_image_age', value=0.0),
                Parameter('require_cuda', value=False),
            ],
        )
        self.client_node = Node('test_client', context=self.context)

        self.bridge = CvBridge()
        self.camera = self.client_node.create_publisher(
            Image, IMAGE_TOPIC, qos_profile_sensor_data
        )
        self.trajectories = []
        self.statuses = []
        self.client_node.create_subscription(
            VlaTrajectory,
            'vla/trajectory',
            lambda msg: self.trajectories.append(msg),
            1,
        )
        self.client_node.create_subscription(
            VlaStatus,
            'vla/status',
            lambda msg: self.statuses.append(msg),
            10,
        )
        self.action_client = ActionClient(
            self.client_node, TrackTarget, 'vla/track_target'
        )

        self.executor = MultiThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.executor.add_node(self.client_node)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()

        self._frame_index = 0
        self._camera_stop = threading.Event()
        self._camera_thread = threading.Thread(
            target=self._publish_frames, daemon=True
        )

    def _spin(self):
        """
        Spin on the executor's own thread pool.

        A hand-rolled spin_once loop services every callback from one thread,
        which lets subscription queues drain far behind real time.
        """
        try:
            self.executor.spin()
        except Exception:
            pass

    def _publish_frames(self):
        """Publish distinct synthetic frames at roughly 100 Hz."""
        while not self._camera_stop.is_set():
            rng = np.random.default_rng(self._frame_index)
            frame = rng.integers(0, 255, (48, 64, 3), dtype=np.uint8)
            message = self.bridge.cv2_to_imgmsg(frame, encoding='rgb8')
            message.header.stamp = \
                self.client_node.get_clock().now().to_msg()
            self.camera.publish(message)
            self._frame_index += 1
            time.sleep(0.01)

    def start_camera(self):
        """Begin publishing frames."""
        self._camera_thread.start()

    def send_goal(self, instruction):
        """Send a goal and return its accepted handle."""
        assert self.action_client.wait_for_server(timeout_sec=10.0), \
            'the action server never came up'
        future = self.action_client.send_goal_async(
            TrackTarget.Goal(instruction=instruction)
        )
        handle = _await(future, timeout=10.0)
        assert handle.accepted, 'the goal was rejected'
        return handle

    def shutdown(self):
        """Tear everything down in the reverse order it was built."""
        self._camera_stop.set()
        if self._camera_thread.is_alive():
            self._camera_thread.join(timeout=2.0)
        self._stop.set()
        self.executor.shutdown()
        self._thread.join(timeout=5.0)
        self.node.destroy_node()
        self.client_node.destroy_node()
        rclpy.shutdown(context=self.context)


def _await(future, timeout):
    """Wait for a future that a background executor is servicing."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if future.done():
            return future.result()
        time.sleep(0.01)
    raise AssertionError('future did not complete in time')


def _wait_until(predicate, timeout, message):
    """Poll until a predicate holds, or fail with a readable message."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.02)
    raise AssertionError(f'timed out waiting for {message}')


@pytest.fixture
def harness():
    """Provide a running harness and guarantee its teardown."""
    running = Harness()
    try:
        yield running
    finally:
        running.shutdown()


def test_no_trajectory_is_published_while_idle(harness):
    """Check the node stays silent until a goal starts a task."""
    harness.start_camera()
    time.sleep(0.5)
    assert harness.trajectories == [], \
        'the node published a trajectory without an active task'


def test_goal_produces_warm_up_then_valid_trajectories(harness):
    """
    Check the full path and the agreed warm-up gating.

    Predictions are published from the first frame so an operator can see
    progress, but stay invalid until the backend's history is full.
    """
    harness.start_camera()
    goal = harness.send_goal(INSTRUCTION)

    _wait_until(
        lambda: len(harness.trajectories) >= HISTORY_LENGTH + 2,
        timeout=15.0,
        message='enough trajectories to pass warm-up',
    )

    published = list(harness.trajectories)
    assert [msg.valid for msg in published[:HISTORY_LENGTH - 1]] == \
        [False] * (HISTORY_LENGTH - 1), 'warm-up predictions must be invalid'
    assert published[HISTORY_LENGTH - 1].valid is True, \
        'the prediction that fills the history must become valid'

    sample = published[-1]
    assert sample.backend_name == 'fake'
    assert sample.header.frame_id == 'base_link'
    assert sample.dt == pytest.approx(0.1)
    assert len(sample.waypoints) == 8
    assert (sample.header.stamp.sec, sample.header.stamp.nanosec) != (0, 0), \
        'the trajectory must carry the observation stamp'

    _wait_until(
        lambda: any(
            status.task_state == VlaStatus.TRACKING
            and status.instruction == INSTRUCTION
            for status in harness.statuses
        ),
        timeout=10.0,
        message='a TRACKING status naming the instruction',
    )

    cancel_result = _await(goal.cancel_goal_async(), timeout=10.0)
    assert cancel_result.goals_canceling, 'the cancel request was refused'


def test_a_new_instruction_preempts_the_previous_task(harness):
    """Check a second goal replaces the first and resets the backend."""
    harness.start_camera()
    first = harness.send_goal(INSTRUCTION)
    first_result = first.get_result_async()

    _wait_until(
        lambda: len(harness.trajectories) >= 1,
        timeout=15.0,
        message='the first task to produce a trajectory',
    )

    harness.send_goal(OTHER_INSTRUCTION)

    outcome = _await(first_result, timeout=15.0)
    assert outcome.result.message == 'preempted'

    _wait_until(
        lambda: any(
            status.instruction == OTHER_INSTRUCTION
            for status in harness.statuses
        ),
        timeout=10.0,
        message='the status to report the new instruction',
    )


def test_an_empty_instruction_is_rejected(harness):
    """Check the node refuses a goal that names no target."""
    assert harness.action_client.wait_for_server(timeout_sec=10.0)
    future = harness.action_client.send_goal_async(
        TrackTarget.Goal(instruction='   ')
    )
    handle = _await(future, timeout=10.0)
    assert handle.accepted is False
