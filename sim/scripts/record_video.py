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
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class Recorder(Node):
    """Subscribes to an image topic and writes every frame to a video file."""

    def __init__(self, topic: str, output: str, fps: float):
        """Open the subscription; the writer is sized by the first frame."""
        super().__init__('sim_video_recorder')
        self._bridge = CvBridge()
        self._writer = None
        self._output = output
        self._fps = fps
        self.frames = 0
        self.create_subscription(
            Image, topic, self._on_image, qos_profile_sensor_data
        )
        self.get_logger().info(f'recording {topic} to {output} at {fps} fps')

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
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = Recorder(args.topic, args.output, args.fps)
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
