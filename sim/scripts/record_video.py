#!/usr/bin/env python3
"""
Write a camera topic to an mp4 file.

Exists because the simulator runs headless: there is no display to attach a
GUI to, so the only way to see what the robot did is to render a chase camera
inside the world and record it. Any image topic works, so the same script can
capture the model's own input when that is what needs inspecting.
"""

import argparse
import time

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from vla_tracking_interfaces.msg import VlaTrajectory

#: Size of the bird's-eye panel drawn in the corner, in pixels.
PANEL = 190


class Recorder(Node):
    """Subscribes to an image topic and writes every frame to a video file."""

    def __init__(self, topic: str, output: str, fps: float, overlay: bool):
        """Open the subscription; the writer is sized by the first frame."""
        super().__init__('sim_video_recorder')
        self._bridge = CvBridge()
        self._writer = None
        self._output = output
        self._fps = fps
        self._trajectory = None
        self.frames = 0
        self.create_subscription(
            Image, topic, self._on_image, qos_profile_sensor_data
        )
        if overlay:
            self.create_subscription(
                VlaTrajectory, '/vla/trajectory', self._on_trajectory, 10
            )
        self.get_logger().info(f'recording {topic} to {output} at {fps} fps')

    def _on_trajectory(self, message: VlaTrajectory) -> None:
        """Keep the latest prediction for the overlay."""
        self._trajectory = message

    def _draw_plan(self, frame):
        """
        Draw the latest prediction as a bird's-eye panel.

        Drawn top-down rather than projected into the image because the
        waypoints are normalized: at the speeds this model commands, the whole
        eight-point horizon covers a couple of centimetres of ground and would
        project to a handful of pixels at the bottom of the frame. A panel
        with its own scale shows the shape of the plan, which is the part
        worth seeing.
        """
        message = self._trajectory
        origin_x = frame.shape[1] - PANEL - 12
        origin_y = 12
        panel = frame[origin_y:origin_y + PANEL, origin_x:origin_x + PANEL]
        cv2.rectangle(frame, (origin_x, origin_y),
                      (origin_x + PANEL, origin_y + PANEL), (40, 40, 40), -1)
        cv2.rectangle(frame, (origin_x, origin_y),
                      (origin_x + PANEL, origin_y + PANEL), (200, 200, 200), 1)

        if message is None or not message.waypoints:
            cv2.putText(frame, 'no plan', (origin_x + 12, origin_y + 100),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (160, 160, 160), 1)
            return

        points = np.array(
            [[w.x, w.y] for w in message.waypoints], dtype=np.float64
        )
        # Normalized units; scale so the plan fills the panel whatever its
        # magnitude, and label the scale so nothing is read as metres.
        span = float(np.max(np.abs(points))) or 1e-6
        scale = (PANEL * 0.40) / span

        centre_x = origin_x + PANEL // 2
        base_y = origin_y + PANEL - 28
        cv2.line(frame, (centre_x, origin_y + 6), (centre_x, base_y),
                 (70, 70, 70), 1)

        colour = (90, 220, 90) if message.valid else (90, 90, 220)
        previous = None
        for index, (fwd, lat) in enumerate(points):
            # Forward is up the panel, lateral is left.
            px = int(centre_x - lat * scale)
            py = int(base_y - fwd * scale)
            if previous is not None:
                cv2.line(frame, previous, (px, py), colour, 2)
            cv2.circle(frame, (px, py), 3 if index else 4, colour, -1)
            previous = (px, py)

        label = 'plan' if message.valid else 'plan (not executed)'
        cv2.putText(frame, label, (origin_x + 8, origin_y + PANEL - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, colour, 1)
        cv2.putText(frame, f'span {span:.3f}', (origin_x + 8, origin_y + 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (170, 170, 170), 1)
        del panel

    def _on_image(self, message: Image) -> None:
        """Convert one frame and append it to the file."""
        try:
            frame = self._bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn(
                f'skipping a frame: {type(exc).__name__}: {exc}',
                throttle_duration_sec=5.0,
            )
            return

        if self._writer is None:
            height, width = frame.shape[:2]
            self._writer = cv2.VideoWriter(
                self._output,
                cv2.VideoWriter_fourcc(*'mp4v'),
                self._fps,
                (width, height),
            )
            if not self._writer.isOpened():
                raise RuntimeError(f'could not open {self._output} for writing')
            self.get_logger().info(f'writing {width}x{height} frames')

        self._draw_plan(frame)
        self._writer.write(frame)
        self.frames += 1

    def close(self) -> None:
        """Finalise the file so it is playable."""
        if self._writer is not None:
            self._writer.release()
            self._writer = None


def main() -> None:
    """Record for a fixed duration, then report what was captured."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--topic', default='/sim/third_person/image_raw')
    parser.add_argument('--output', default='sim/output/third_person.mp4')
    parser.add_argument('--fps', type=float, default=20.0)
    parser.add_argument(
        '--duration', type=float, default=30.0,
        help='seconds to record; 0 records until interrupted',
    )
    parser.add_argument(
        '--overlay-trajectory', action='store_true',
        help="draw the model's latest prediction as a bird's-eye panel",
    )
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = Recorder(args.topic, args.output, args.fps,
                    args.overlay_trajectory)
    deadline = time.monotonic() + args.duration if args.duration > 0 else None
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if deadline is not None and time.monotonic() >= deadline:
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        captured = node.frames
        node.get_logger().info(
            f'captured {captured} frames into {args.output}'
        )
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    if captured == 0:
        raise SystemExit(
            f'no frames arrived on {args.topic}; the bridge or the sensor is '
            f'not publishing'
        )


if __name__ == '__main__':
    main()
