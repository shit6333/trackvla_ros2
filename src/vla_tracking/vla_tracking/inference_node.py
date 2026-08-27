"""
ROS 2 node that turns camera frames and an instruction into a trajectory.

The node owns the ROS surface and the task lifecycle; the model lives behind
the backend contract and owns its own temporal state. The node publishes no
velocity command: converting a trajectory into motion is the trajectory
executor's job, so that a model failure can never actuate the base directly.
"""

import threading
import time
from typing import Optional

from cv_bridge import CvBridge
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import (
    MutuallyExclusiveCallbackGroup,
    ReentrantCallbackGroup,
)
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from vla_tracking.backend_interface import BackendError, Observation
from vla_tracking.backend_loader import load_backend
from vla_tracking.image_buffer import LatestFrame

from vla_tracking_interfaces.action import TrackTarget
from vla_tracking_interfaces.msg import (
    VlaStatus,
    VlaTrajectory,
    Waypoint2D,
)

NANOSECONDS_PER_SECOND = 1_000_000_000


class VlaInferenceNode(Node):
    """Serves TrackTarget goals by driving a VLA backend over camera frames."""

    def __init__(self, **kwargs):
        """Declare parameters, load the backend, and open the ROS surface."""
        super().__init__('vla_inference_node', **kwargs)

        self.declare_parameter('backend', 'fake')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('inference_rate', 10.0)
        self.declare_parameter('max_image_age', 0.5)
        self.declare_parameter('status_rate', 5.0)
        # Backend configuration. Empty strings mean "fall back to the
        # environment", so no developer path is ever baked into a default.
        self.declare_parameter('model_dir', '')
        self.declare_parameter('source_path', '')
        self.declare_parameter('require_cuda', True)
        self.declare_parameter('history_length', 0)

        self._base_frame = self._string_parameter('base_frame')
        self._inference_period = 1.0 / max(
            self.get_parameter('inference_rate').value, 1e-3
        )
        self._max_image_age_ns = int(
            self.get_parameter('max_image_age').value * NANOSECONDS_PER_SECOND
        )

        self._bridge = CvBridge()
        self._latest = LatestFrame()

        self._state_lock = threading.Lock()
        self._task_state = VlaStatus.IDLE
        self._instruction = ''
        self._predictions_published = 0
        self._last_inference_duration = 0.0
        self._last_observation_stamp_ns = 0
        self._error_message = ''

        self._backend_name = self._string_parameter('backend')
        self._backend = None
        self._backend_ready = False
        self._configure_backend()

        sensor_group = MutuallyExclusiveCallbackGroup()
        timer_group = MutuallyExclusiveCallbackGroup()
        action_group = ReentrantCallbackGroup()

        self._trajectory_publisher = self.create_publisher(
            VlaTrajectory, 'vla/trajectory', 1
        )
        self._status_publisher = self.create_publisher(
            VlaStatus, 'vla/status', 10
        )
        self._image_subscription = self.create_subscription(
            Image,
            self._string_parameter('image_topic'),
            self._on_image,
            qos_profile_sensor_data,
            callback_group=sensor_group,
        )

        self._goal_lock = threading.Lock()
        self._active_goal = None
        self._action_server = ActionServer(
            self,
            TrackTarget,
            'vla/track_target',
            execute_callback=self._execute,
            goal_callback=self._on_goal,
            handle_accepted_callback=self._on_goal_accepted,
            cancel_callback=self._on_cancel,
            callback_group=action_group,
        )

        self._status_timer = self.create_timer(
            1.0 / max(self.get_parameter('status_rate').value, 1e-3),
            self._publish_status,
            callback_group=timer_group,
        )

        self.get_logger().info(
            f'backend {self._backend_name!r} ready={self._backend_ready}, '
            f'inference rate '
            f'{self.get_parameter("inference_rate").value:.1f} Hz'
        )

    # -- setup ------------------------------------------------------------

    def _string_parameter(self, name: str) -> str:
        """Read a string parameter."""
        return str(self.get_parameter(name).value)

    def _configure_backend(self) -> None:
        """
        Load and configure the backend, surviving a failure as ERROR state.

        A model that cannot load must not take the node down: the node still
        has to answer goals so an operator can see why nothing is tracking.
        """
        config = {
            'model_dir': self._string_parameter('model_dir') or None,
            'source_path': self._string_parameter('source_path') or None,
            'require_cuda': bool(self.get_parameter('require_cuda').value),
            'frame_id': self._base_frame,
        }
        history_length = int(self.get_parameter('history_length').value)
        if history_length > 0:
            config['history_length'] = history_length

        try:
            self._backend = load_backend(self._backend_name)
            self._backend.configure(config)
            self._backend_ready = True
        except Exception as exc:
            self._backend = None
            self._backend_ready = False
            with self._state_lock:
                self._task_state = VlaStatus.ERROR
                self._error_message = (
                    f'backend {self._backend_name!r} failed to configure: '
                    f'{type(exc).__name__}: {exc}'
                )
            self.get_logger().error(self._error_message)

    # -- sensor ------------------------------------------------------------

    def _on_image(self, message: Image) -> None:
        """
        Store the newest frame. Must stay cheap: inference happens elsewhere.

        cv_bridge does the conversion rather than a manual reshape because
        sensor_msgs/Image allows a row stride larger than width * channels,
        and a naive reshape corrupts every frame from such a camera.
        """
        try:
            rgb = self._bridge.imgmsg_to_cv2(message, desired_encoding='rgb8')
        except Exception as exc:
            self.get_logger().warn(
                f'dropping frame with encoding {message.encoding!r}: '
                f'{type(exc).__name__}: {exc}',
                throttle_duration_sec=5.0,
            )
            return

        stamp_ns = (
            message.header.stamp.sec * NANOSECONDS_PER_SECOND
            + message.header.stamp.nanosec
        )
        if stamp_ns == 0:
            stamp_ns = self.get_clock().now().nanoseconds
        self._latest.put(rgb, stamp_ns)

    # -- action lifecycle ---------------------------------------------------

    def _on_goal(self, goal_request) -> GoalResponse:
        """Accept any goal that names a target."""
        if not goal_request.instruction.strip():
            self.get_logger().warn('rejecting a goal with an empty instruction')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _on_goal_accepted(self, goal_handle) -> None:
        """Preempt any running task, then start this one."""
        with self._goal_lock:
            previous = self._active_goal
            self._active_goal = goal_handle
        if previous is not None and previous.is_active:
            self.get_logger().info('preempting the active tracking task')
            previous.abort()
        goal_handle.execute()

    def _on_cancel(self, goal_handle) -> CancelResponse:
        """Accept every cancellation request."""
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle) -> TrackTarget.Result:
        """
        Run one tracking task until it is cancelled, preempted, or fails.

        Every exit path publishes a final status and leaves the backend with no
        residue from this task, so the next goal starts clean.
        """
        instruction = goal_handle.request.instruction
        self.get_logger().info(f'tracking task started: {instruction!r}')

        if not self._backend_ready:
            return self._finish(
                goal_handle,
                outcome='abort',
                message=self._error_message or 'no backend is configured',
            )

        try:
            self._backend.reset(instruction)
        except Exception as exc:
            return self._finish(
                goal_handle,
                outcome='abort',
                message=f'backend reset failed: {type(exc).__name__}: {exc}',
            )

        with self._state_lock:
            self._task_state = VlaStatus.WARMING_UP
            self._instruction = instruction
            self._predictions_published = 0
            self._error_message = ''
        self._latest.clear()

        published = 0
        next_due = time.monotonic()
        while self.context.ok():
            if not goal_handle.is_active:
                return self._finish(
                    goal_handle,
                    outcome='preempted',
                    message='preempted',
                    published=published,
                )
            if goal_handle.is_cancel_requested:
                return self._finish(
                    goal_handle,
                    outcome='cancel',
                    message='cancelled',
                    published=published,
                )

            now = time.monotonic()
            if now < next_due:
                time.sleep(min(next_due - now, self._inference_period))
                continue
            next_due = now + self._inference_period

            frame = self._latest.take()
            if frame is None:
                continue

            rgb, stamp_ns = frame
            age_ns = self.get_clock().now().nanoseconds - stamp_ns
            if 0 < self._max_image_age_ns < age_ns:
                self.get_logger().warn(
                    f'skipping a frame {age_ns / 1e6:.0f} ms old',
                    throttle_duration_sec=5.0,
                )
                continue

            try:
                started = time.perf_counter()
                prediction = self._backend.infer(
                    Observation(
                        rgb=rgb,
                        stamp_ns=stamp_ns,
                        instruction=instruction,
                    )
                )
                duration = time.perf_counter() - started
            except BackendError as exc:
                return self._finish(
                    goal_handle,
                    outcome='abort',
                    message=f'inference failed: {exc}',
                    published=published,
                )

            if self._publish_prediction(prediction, duration, goal_handle):
                published += 1
                goal_handle.publish_feedback(
                    TrackTarget.Feedback(status=self._status_message())
                )

        return self._finish(
            goal_handle,
            outcome='abort',
            message='node is shutting down',
            published=published,
        )

    def _finish(
        self,
        goal_handle,
        outcome: str,
        message: str,
        published: int = 0,
    ):
        """
        Settle the goal exactly once and return to a safe idle state.

        A preempted task must not touch the shared task state, because the
        task that replaced it has already installed its own instruction and
        counters by the time this runs.
        """
        with self._goal_lock:
            still_current = self._active_goal is goal_handle
            if still_current:
                self._active_goal = None

        if still_current:
            with self._state_lock:
                if outcome == 'abort':
                    self._task_state = VlaStatus.ERROR
                    self._error_message = message
                else:
                    self._task_state = VlaStatus.STOPPED
                self._instruction = ''

        status = self._status_message()
        if goal_handle.is_active:
            if outcome == 'cancel':
                goal_handle.canceled()
            elif outcome == 'abort':
                goal_handle.abort()
            else:
                goal_handle.succeed()

        self.get_logger().info(f'tracking task ended: {message}')
        self._publish_status()
        return TrackTarget.Result(
            final_status=status,
            predictions_published=published,
            message=message,
        )

    # -- publishing ---------------------------------------------------------

    def _publish_prediction(
        self, prediction, duration: float, goal_handle
    ) -> bool:
        """
        Publish a trajectory and record the state it implies.

        Returns False without publishing when the task was preempted while
        this prediction was being computed, so a superseded target can never
        emit a trajectory that the executor would act on.
        """
        with self._goal_lock:
            if self._active_goal is not goal_handle:
                return False

        message = VlaTrajectory()
        message.header.stamp.sec = int(
            prediction.stamp_ns // NANOSECONDS_PER_SECOND
        )
        message.header.stamp.nanosec = int(
            prediction.stamp_ns % NANOSECONDS_PER_SECOND
        )
        message.header.frame_id = prediction.frame_id
        message.backend_name = self._backend.name
        message.waypoints = [
            Waypoint2D(x=float(row[0]), y=float(row[1]), theta=float(row[2]))
            for row in prediction.waypoints
        ]
        message.dt = float(prediction.dt)
        message.valid = bool(prediction.valid)
        message.status = prediction.status
        self._trajectory_publisher.publish(message)

        with self._state_lock:
            self._task_state = (
                VlaStatus.WARMING_UP if prediction.warming_up
                else VlaStatus.TRACKING
            )
            self._predictions_published += 1
            self._last_inference_duration = duration
            self._last_observation_stamp_ns = prediction.stamp_ns
        return True

    def _status_message(self) -> VlaStatus:
        """Snapshot the current state as a status message."""
        message = VlaStatus()
        message.header.stamp = self.get_clock().now().to_msg()
        with self._state_lock:
            message.task_state = self._task_state
            message.instruction = self._instruction
            message.predictions_published = self._predictions_published
            message.last_inference_duration = float(
                self._last_inference_duration
            )
            message.last_observation_stamp.sec = int(
                self._last_observation_stamp_ns // NANOSECONDS_PER_SECOND
            )
            message.last_observation_stamp.nanosec = int(
                self._last_observation_stamp_ns % NANOSECONDS_PER_SECOND
            )
            message.error_message = self._error_message
        message.backend_name = self._backend_name
        message.backend_ready = self._backend_ready
        return message

    def _publish_status(self) -> None:
        """Publish the current state on the status topic."""
        self._status_publisher.publish(self._status_message())

    # -- teardown -----------------------------------------------------------

    def destroy_node(self) -> bool:
        """Shut the backend down before tearing the ROS surface down."""
        if self._backend is not None:
            try:
                self._backend.shutdown()
            except Exception as exc:
                self.get_logger().warn(
                    f'backend shutdown raised {type(exc).__name__}: {exc}'
                )
            self._backend = None
        return super().destroy_node()


def main(args: Optional[list] = None) -> None:
    """Spin the inference node on a multi-threaded executor."""
    rclpy.init(args=args)
    node = VlaInferenceNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
