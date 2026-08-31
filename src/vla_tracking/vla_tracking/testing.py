"""
Reusable fixtures for exercising the pipeline over a real ROS graph.

Shipped with the package rather than kept beside the tests so that adapter
packages can drive the same pipeline against real weights without copying the
scaffolding, and so a new backend can be integration-tested the way the fake
one is.
"""

import threading
import time
from typing import Callable, List, Optional

from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from vla_tracking.backend_conformance import make_frame

from vla_tracking_interfaces.action import TrackTarget
from vla_tracking_interfaces.msg import VlaStatus, VlaTrajectory

try:
    from geometry_msgs.msg import Twist
except ImportError:  # pragma: no cover - geometry_msgs is a hard dependency
    Twist = None


def wait_until(
    predicate: Callable[[], bool],
    timeout: float,
    description: str,
    interval: float = 0.02,
) -> None:
    """Poll until a predicate holds, failing with a readable message."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(interval)
    raise AssertionError(f'timed out after {timeout:.1f}s waiting for {description}')


def await_future(future, timeout: float):
    """Wait for a future that a background executor is servicing."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if future.done():
            return future.result()
        time.sleep(0.01)
    raise AssertionError(f'future did not complete within {timeout:.1f}s')


class SpinningGraph:
    """
    Own an isolated ROS context and spin the nodes added to it.

    Every harness uses its own context so tests cannot disturb one another,
    and every node gets its own executor and thread.

    That last part is not a style choice. Sharing one executor between a node
    that publishes on a timer and a node that observes it starves the
    observer: measured against a 50 Hz publisher, a shared executor delivered
    3.5 Hz while separate ones delivered the full 50. A test built on the
    shared arrangement measures the harness rather than the node, and does so
    intermittently, which reads as a flaky product rather than a flaky test.
    """

    def __init__(self):
        """Create and initialise an isolated context."""
        self.context = Context()
        rclpy.init(context=self.context)
        self._nodes: List[Node] = []
        self._executors: List[MultiThreadedExecutor] = []
        self._threads: List[threading.Thread] = []

    def add(self, node: Node) -> Node:
        """Register a node, giving it an executor of its own."""
        # Capped deliberately: the default is one thread per CPU,
        # so two nodes opened 48 for a handful of callbacks and
        # the contention showed up as scheduling stalls long
        # enough to skip an entire trajectory segment.
        executor = MultiThreadedExecutor(
            num_threads=4, context=self.context
        )
        executor.add_node(node)
        self._nodes.append(node)
        self._executors.append(executor)
        return node

    def start(self) -> None:
        """Begin spinning every node on its own background thread."""
        for executor in self._executors:
            thread = threading.Thread(
                target=self._spin, args=(executor,), daemon=True
            )
            thread.start()
            self._threads.append(thread)

    @staticmethod
    def _spin(executor: MultiThreadedExecutor) -> None:
        """Spin one executor until it is shut down."""
        try:
            executor.spin()
        except Exception:
            pass

    def shutdown(self) -> None:
        """Stop spinning and destroy every node, tolerating partial setup."""
        for executor in self._executors:
            executor.shutdown()
        for thread in self._threads:
            thread.join(timeout=5.0)
        for node in reversed(self._nodes):
            try:
                node.destroy_node()
            except Exception:
                pass
        try:
            rclpy.shutdown(context=self.context)
        except Exception:
            pass


class FakeCamera(Node):
    """Publishes deterministic synthetic RGB frames at a fixed rate."""

    def __init__(
        self,
        context: Context,
        topic: str = '/camera/image_raw',
        shape=(240, 320),
        rate: float = 30.0,
        name: str = 'fake_camera',
    ):
        """Create the publisher; frames start flowing only after start()."""
        super().__init__(name, context=context)
        self._publisher = self.create_publisher(
            Image, topic, qos_profile_sensor_data
        )
        self._bridge = CvBridge()
        self._shape = shape
        self._period = 1.0 / max(rate, 1e-3)
        self._index = 0
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    def start(self) -> None:
        """Begin publishing frames on a dedicated thread."""
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self) -> None:
        """Publish a new frame every period until stopped."""
        while not self._stop.is_set():
            frame = make_frame(self._index, self._shape)
            message = self._bridge.cv2_to_imgmsg(frame, encoding='rgb8')
            message.header.stamp = self.get_clock().now().to_msg()
            self._publisher.publish(message)
            self._index += 1
            time.sleep(self._period)

    @property
    def frames_published(self) -> int:
        """Return how many frames have been sent."""
        return self._index

    def stop(self) -> None:
        """Stop publishing and join the thread."""
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)


class PipelineObserver(Node):
    """Records what the pipeline produces and can start tracking tasks."""

    def __init__(self, context: Context, name: str = 'pipeline_observer'):
        """Subscribe to every pipeline output and open an action client."""
        super().__init__(name, context=context)
        self.trajectories: List[VlaTrajectory] = []
        self.statuses: List[VlaStatus] = []
        self.commands: List = []

        self.create_subscription(
            VlaTrajectory,
            'vla/trajectory',
            lambda msg: self.trajectories.append(msg),
            10,
        )
        self.create_subscription(
            VlaStatus, 'vla/status', lambda msg: self.statuses.append(msg), 10
        )
        if Twist is not None:
            self.create_subscription(
                Twist, 'cmd_vel', lambda msg: self.commands.append(msg), 10
            )
        self.action_client = ActionClient(
            self, TrackTarget, 'vla/track_target'
        )

    def start_task(self, instruction: str, timeout: float = 20.0):
        """Send a goal and return its accepted handle."""
        if not self.action_client.wait_for_server(timeout_sec=timeout):
            raise AssertionError('the tracking action server never came up')
        handle = await_future(
            self.action_client.send_goal_async(
                TrackTarget.Goal(instruction=instruction)
            ),
            timeout=timeout,
        )
        if not handle.accepted:
            raise AssertionError(f'the goal {instruction!r} was rejected')
        return handle

    def moving_commands(self) -> List:
        """Return the recorded commands that would move the base."""
        return [
            command for command in self.commands
            if command.linear.x != 0.0
            or command.linear.y != 0.0
            or command.angular.z != 0.0
        ]

    def valid_trajectories(self) -> List[VlaTrajectory]:
        """Return the recorded trajectories that were executable."""
        return [msg for msg in self.trajectories if msg.valid]


def frame_like(seed: int, shape=(240, 320)) -> np.ndarray:
    """Build a deterministic RGB frame, re-exported for adapter tests."""
    return make_frame(seed, shape)
