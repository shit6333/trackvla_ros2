"""
Everything the tracking pipeline needs against the simulator, in one command.

Brings up the two runtime nodes with the simulation parameter file, plus the
pieces that exist only to make the run observable: the Foxglove bridge and the
two marker publishers.

They are launched together rather than started as separate containers. All of
them join the same ROS graph, and with host networking a separate container
buys no isolation for a node that is already sharing the DNS domain; it only
adds something else to remember to stop. The simulator stays in its own
container because it is a different image.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

SIM_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _as_bool(value):
    """Interpret a launch argument as a boolean."""
    return str(value).strip().lower() in ('true', '1', 'yes', 'on')


def _setup(context, *args, **kwargs):
    """Build the node list once the launch arguments are known."""
    backend = LaunchConfiguration('backend').perform(context)
    port = LaunchConfiguration('foxglove_port').perform(context)
    with_viewers = _as_bool(
        LaunchConfiguration('enable_visualization').perform(context)
    )

    params_file = os.path.join(SIM_ROOT, 'config', 'tracking_sim.yaml')
    actions = [
        LogInfo(msg=f'[sim] backend={backend}, parameters from {params_file}'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('vla_tracking'),
                    'launch', 'tracking.launch.py',
                )
            ),
            launch_arguments={
                'backend': backend,
                'params_file': params_file,
            }.items(),
        ),
    ]

    if not with_viewers:
        return actions

    actions.append(
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='screen',
            parameters=[{'port': int(port), 'address': '0.0.0.0'}],
        )
    )
    for script in ('publish_scene_markers.py', 'publish_trajectory_markers.py'):
        actions.append(
            ExecuteProcess(
                cmd=['python3', os.path.join(SIM_ROOT, 'scripts', script)],
                output='screen',
            )
        )
    actions.append(
        LogInfo(msg=f'[sim] Foxglove at ws://<this host>:{port}, display frame '
                    f'"tracking"')
    )
    return actions


def generate_launch_description() -> LaunchDescription:
    """Declare the arguments and defer construction."""
    return LaunchDescription([
        DeclareLaunchArgument('backend', default_value='omtrackvla'),
        DeclareLaunchArgument('foxglove_port', default_value='8765'),
        DeclareLaunchArgument(
            'enable_visualization',
            default_value='true',
            description=(
                'Run the Foxglove bridge and the marker publishers. Turn off '
                'for a headless measurement run, where they only add load.'
            ),
        ),
        OpaqueFunction(function=_setup),
    ])
