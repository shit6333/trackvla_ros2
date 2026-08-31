"""
Bring up the headless simulator and the ROS bridge.

Run from the workspace root inside the gazebo container::

    ros2 launch sim/launch/sim.launch.py

There is no GUI. The simulator renders offscreen, and what happened is
observed through the bridged topics or recorded with sim/scripts/record_video.py.
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

SIM_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def generate_launch_description() -> LaunchDescription:
    """Start the Gazebo server and the parameter bridge."""
    world = LaunchConfiguration('world')
    bridge_config = LaunchConfiguration('bridge_config')

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(SIM_ROOT, 'worlds', 'tracking.sdf'),
            description='World to load.',
        ),
        DeclareLaunchArgument(
            'bridge_config',
            default_value=os.path.join(SIM_ROOT, 'config', 'bridge.yaml'),
            description='Topics to bridge between Gazebo and ROS.',
        ),
        LogInfo(msg=f'[sim] resources under {SIM_ROOT}'),

        ExecuteProcess(
            # -s runs the server with no GUI, -r starts stepping immediately,
            # and --headless-rendering is what lets the camera sensors render
            # with no display attached.
            cmd=[
                'gz', 'sim', '-s', '-r', '--headless-rendering', '-v', '1',
                world,
            ],
            output='screen',
            additional_env={
                'GZ_SIM_RESOURCE_PATH': os.pathsep.join([
                    os.path.join(SIM_ROOT, 'models'),
                    os.path.join(SIM_ROOT, 'worlds'),
                    os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
                ]),
            },
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ros_gz_bridge',
            output='screen',
            parameters=[{'config_file': bridge_config}],
        ),
    ])
