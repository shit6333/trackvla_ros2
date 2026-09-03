#!/usr/bin/env python3
"""
Publish the world's static geometry, and the robot's body, as markers.

Foxglove's 3D panel draws transforms and markers; it cannot read a Gazebo
world, so without this the panel shows an empty grid even while everything is
running. The markers are derived from the world file rather than hand-written,
so editing the world moves them too.

The walking actor is deliberately absent. Gazebo does not publish an actor's
pose anywhere: it appears in neither pose topic, nor the serialized world
state, nor the model list, because actors are rendering entities rather than
simulated bodies. Computing its pose from the trajectory script in the world
file was considered and rejected: it would duplicate the world definition
here, and a later edit to those waypoints would leave this drawing a person
somewhere they are not, which is worse than not drawing them. The person is
visible in the camera images.
"""

import argparse
import math
import pathlib
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from visualization_msgs.msg import Marker, MarkerArray

#: Marker type constants, spelled out because the numeric values read as noise.
CUBE = Marker.CUBE
CYLINDER = Marker.CYLINDER

#: Ground planes are infinite in SDF; drawn as a thin slab this many metres wide.
GROUND_EXTENT = 40.0
GROUND_THICKNESS = 0.02


def parse_pose(element):
    """Read an SDF pose element into x, y, z, roll, pitch, yaw."""
    if element is None or not (element.text or '').strip():
        return (0.0,) * 6
    values = [float(v) for v in element.text.split()]
    values += [0.0] * (6 - len(values))
    return tuple(values[:6])


def yaw_to_quaternion(yaw):
    """Convert a yaw to a quaternion, which is what a Marker wants."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def marker_from_visual(model, visual, marker_id, frame_id):
    """Build one marker from a model's visual, or None for shapes not drawn."""
    geometry = visual.find('geometry')
    if geometry is None:
        return None

    marker = Marker()
    marker.header.frame_id = frame_id
    marker.ns = 'world'
    marker.id = marker_id
    marker.action = Marker.ADD

    box = geometry.find('box')
    cylinder = geometry.find('cylinder')
    plane = geometry.find('plane')

    if box is not None:
        size = [float(v) for v in box.find('size').text.split()]
        marker.type = CUBE
        marker.scale.x, marker.scale.y, marker.scale.z = size
    elif cylinder is not None:
        radius = float(cylinder.find('radius').text)
        length = float(cylinder.find('length').text)
        marker.type = CYLINDER
        marker.scale.x = marker.scale.y = radius * 2.0
        marker.scale.z = length
    elif plane is not None:
        marker.type = CUBE
        marker.scale.x = marker.scale.y = GROUND_EXTENT
        marker.scale.z = GROUND_THICKNESS
    else:
        return None

    x, y, z, _, _, yaw = parse_pose(model.find('pose'))
    vx, vy, vz, _, _, _ = parse_pose(visual.find('pose'))
    marker.pose.position.x = x + vx
    marker.pose.position.y = y + vy
    marker.pose.position.z = z + vz
    if plane is not None:
        marker.pose.position.z -= GROUND_THICKNESS / 2.0
    (marker.pose.orientation.x, marker.pose.orientation.y,
     marker.pose.orientation.z, marker.pose.orientation.w) = yaw_to_quaternion(yaw)

    material = visual.find('material')
    diffuse = material.find('diffuse') if material is not None else None
    if diffuse is not None and (diffuse.text or '').strip():
        r, g, b, a = ([float(v) for v in diffuse.text.split()] + [1.0] * 4)[:4]
    else:
        r, g, b, a = 0.6, 0.6, 0.6, 1.0
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = r, g, b, a
    return marker


def build_markers(world_path, frame_id):
    """Read the world and return a marker for every static visual in it."""
    root = ET.parse(world_path).getroot()
    world = root.find('world')
    markers = MarkerArray()
    marker_id = 0
    for model in world.findall('model'):
        static = model.find('static')
        if static is None or (static.text or '').strip().lower() != 'true':
            # Non-static models move, so drawing them from the file would
            # freeze them wherever they happened to be written.
            continue
        for link in model.findall('link'):
            for visual in link.findall('visual'):
                marker = marker_from_visual(model, visual, marker_id, frame_id)
                if marker is not None:
                    markers.markers.append(marker)
                    marker_id += 1
    return markers


def build_robot_marker(frame_id='turtlebot3_burger/base'):
    """Draw the chassis in the robot's own frame so TF carries it around."""
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.ns = 'robot'
    marker.id = 0
    marker.type = CUBE
    marker.action = Marker.ADD
    marker.scale.x, marker.scale.y, marker.scale.z = 0.4, 0.3, 0.15
    marker.pose.orientation.w = 1.0
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = (
        0.25, 0.4, 0.7, 1.0
    )
    return marker


def main():
    """Publish the scene once and keep it available to late subscribers."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--world',
        default=str(pathlib.Path(__file__).resolve().parent.parent
                    / 'worlds' / 'tracking.sdf'),
    )
    parser.add_argument(
        '--frame', default='tracking',
        help="Frame the static geometry is drawn in, which is the world's own "
             'name. Deliberately not odom: wheel odometry drifts, and drawing '
             'a fixed scene in a drifting frame puts the robot metres away '
             'from the posts it is driving past.',
    )
    parser.add_argument(
        '--robot-frame', default='turtlebot3_burger/base',
        help='Frame the chassis marker rides on, as named by the pose '
             'publisher plugin.',
    )
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = Node('scene_markers')

    # Transient local, so a viewer that connects later still receives the
    # scene instead of an empty panel.
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    publisher = node.create_publisher(MarkerArray, 'scene_markers', qos)

    markers = build_markers(args.world, args.frame)
    markers.markers.append(build_robot_marker(args.robot_frame))
    node.get_logger().info(
        f'publishing {len(markers.markers)} markers from {args.world}'
    )

    def republish():
        stamp = node.get_clock().now().to_msg()
        for marker in markers.markers:
            marker.header.stamp = stamp
        publisher.publish(markers)

    republish()
    node.create_timer(2.0, republish)
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
