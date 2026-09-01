#!/usr/bin/env python3
"""
Draw the model's predicted trajectory in the 3D view.

A VlaTrajectory carries normalized waypoints, not metres, so it cannot be
plotted directly: the numbers are a command integrated with the checkpoint's
bookkeeping constant, and a viewer that treated them as positions would draw a
path a few centimetres long regardless of how fast the robot was told to go.

The conversion is imported from the runtime rather than repeated here. Drawing
uses `trajectory_to_segments` and `scale_segment`, the same functions the
executor uses, so the line shown is the path the robot was actually commanded
along. A separate implementation would be free to drift from it, and a
visualization that quietly disagrees with the controller is worse than none.

The first segment is drawn brightly and the rest dimly, because with the
default `waypoints_to_execute` of 1 only that segment is ever driven; the
remainder is what the model intended, not what happened.
"""

import argparse
import math

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

from vla_tracking.trajectory_conversion import scale_segment, trajectory_to_segments

from vla_tracking_interfaces.msg import VlaTrajectory


def integrate(segments):
    """
    Turn scaled velocity segments back into a path in metres.

    Each segment is a constant velocity held for its duration, expressed in
    the frame of the pose that starts it, so the path is rebuilt by walking
    them in order rather than by summing the raw numbers.
    """
    x = y = theta = 0.0
    path = [(x, y)]
    for segment in segments:
        forward = segment.linear_x * segment.duration
        lateral = segment.linear_y * segment.duration
        x += math.cos(theta) * forward - math.sin(theta) * lateral
        y += math.sin(theta) * forward + math.cos(theta) * lateral
        theta += segment.angular_z * segment.duration
        path.append((x, y))
    return path


class TrajectoryMarkers(Node):
    """Republishes predictions as markers a 3D viewer can draw."""

    def __init__(self, scales, height, frame):
        """Subscribe to predictions and publish the drawn form of them."""
        super().__init__('trajectory_markers')
        self._scales = scales
        self._height = height
        self._frame = frame
        self._publisher = self.create_publisher(
            MarkerArray, 'vla/trajectory_markers', 10
        )
        self.create_subscription(
            VlaTrajectory, 'vla/trajectory', self._on_trajectory, 10
        )
        self.get_logger().info(
            f'drawing with scales {scales[0]:.2f} / {scales[1]:.2f} m/s and '
            f'{scales[2]:.2f} rad/s per unit'
        )

    def _on_trajectory(self, message: VlaTrajectory) -> None:
        """Convert one prediction and publish it."""
        if len(message.waypoints) < 2 or message.dt <= 0.0:
            return

        waypoints = [(w.x, w.y, w.theta) for w in message.waypoints]
        try:
            segments = trajectory_to_segments(
                waypoints, message.dt, len(waypoints) - 1
            )
        except ValueError:
            return
        scaled = [scale_segment(s, *self._scales) for s in segments]
        path = integrate(scaled)

        stamp = self.get_clock().now().to_msg()
        # The message names base_link, which is what a real robot would call
        # it. The simulator's pose publisher scopes its frames by model, so a
        # viewer needs the scoped name instead.
        frame = self._frame or message.header.frame_id or 'base_link'
        # Green when the executor would accept it, red when it would not, so
        # a warm-up or failed prediction is visibly different from a live one.
        colour = (0.2, 0.9, 0.3) if message.valid else (0.9, 0.25, 0.2)

        markers = MarkerArray()
        markers.markers.append(
            self._line('plan', 0, path[1:], stamp, frame, colour, 0.35, 0.02)
        )
        markers.markers.append(
            self._line('plan', 1, path[:2], stamp, frame, colour, 1.0, 0.035)
        )
        markers.markers.append(
            self._points('plan', 2, path, stamp, frame, colour)
        )
        self._publisher.publish(markers)

    def _line(self, ns, marker_id, path, stamp, frame, colour, alpha, width):
        """Build a line strip through the given points."""
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = frame
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = width
        marker.pose.orientation.w = 1.0
        marker.color.r, marker.color.g, marker.color.b = colour
        marker.color.a = alpha
        marker.points = [self._point(x, y) for x, y in path]
        return marker

    def _points(self, ns, marker_id, path, stamp, frame, colour):
        """Build a sphere at every waypoint."""
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = frame
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.scale.x = marker.scale.y = marker.scale.z = 0.05
        marker.pose.orientation.w = 1.0
        marker.color.r, marker.color.g, marker.color.b = colour
        marker.color.a = 0.9
        marker.points = [self._point(x, y) for x, y in path]
        return marker

    def _point(self, x, y):
        """Lift a planar point off the ground so it is not hidden by it."""
        from geometry_msgs.msg import Point
        return Point(x=float(x), y=float(y), z=float(self._height))


def main():
    """Run the marker publisher."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--linear-scale', type=float, default=3.75)
    parser.add_argument('--lateral-scale', type=float, default=2.5)
    parser.add_argument('--angular-scale', type=float, default=1.57)
    parser.add_argument('--height', type=float, default=0.12)
    parser.add_argument(
        '--frame', default='test_robot/base_link',
        help='Override the frame the plan is drawn in. Empty uses the frame '
             'named in the message.',
    )
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = TrajectoryMarkers(
        (args.linear_scale, args.lateral_scale, args.angular_scale),
        args.height,
        args.frame,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
