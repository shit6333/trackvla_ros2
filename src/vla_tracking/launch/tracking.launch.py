"""
Bring up the tracking pipeline and, optionally, the Nav2 command chain.

Typical invocations::

    ros2 launch vla_tracking tracking.launch.py
    ros2 launch vla_tracking tracking.launch.py backend:=omtrackvla
    ros2 launch vla_tracking tracking.launch.py enable_velocity_smoother:=true
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

from vla_tracking.cmd_vel_routing import (
    COLLISION_MONITOR,
    plan_cmd_vel_chain,
    VELOCITY_SMOOTHER,
)

PACKAGE = 'vla_tracking'


def _config(name: str) -> str:
    """Resolve a configuration file shipped with the package."""
    return os.path.join(get_package_share_directory(PACKAGE), 'config', name)


def _as_bool(value: str) -> bool:
    """Interpret a launch argument as a boolean."""
    return str(value).strip().lower() in ('true', '1', 'yes', 'on')


def _setup(context, *args, **kwargs):
    """Build the node list once the launch arguments are known."""
    params_file = LaunchConfiguration('params_file').perform(context)
    backend = LaunchConfiguration('backend').perform(context)
    image_topic = LaunchConfiguration('image_topic').perform(context)
    enable_smoother = _as_bool(
        LaunchConfiguration('enable_velocity_smoother').perform(context)
    )
    enable_monitor = _as_bool(
        LaunchConfiguration('enable_collision_monitor').perform(context)
    )
    use_sim_time = _as_bool(
        LaunchConfiguration('use_sim_time').perform(context)
    )

    plan = plan_cmd_vel_chain(enable_smoother, enable_monitor)

    actions = [
        LogInfo(
            msg=(
                f'[vla_tracking] backend={backend}; '
                f'executor publishes {plan.executor_output!r}; '
                f'{plan.final_publisher} owns cmd_vel; '
                f'chain={list(plan.enabled_names) or "bypassed"}'
            )
        ),
        Node(
            package=PACKAGE,
            executable='vla_inference_node',
            name='vla_inference_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                params_file,
                {
                    'backend': backend,
                    'image_topic': image_topic,
                    'use_sim_time': use_sim_time,
                },
            ],
        ),
        Node(
            package=PACKAGE,
            executable='trajectory_executor_node',
            name='trajectory_executor_node',
            output='screen',
            emulate_tty=True,
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            # With the chain bypassed this resolves to itself, so the executor
            # publishes the final topic directly rather than through a no-op
            # stage.
            remappings=[('cmd_vel', plan.executor_output)],
        ),
    ]

    lifecycle_nodes = []
    for stage in plan.stages:
        if stage.name == VELOCITY_SMOOTHER:
            actions.append(
                Node(
                    package='nav2_velocity_smoother',
                    executable='velocity_smoother',
                    name=VELOCITY_SMOOTHER,
                    output='screen',
                    parameters=[_config('velocity_smoother.yaml')],
                    # The smoother takes its topics by name, not by parameter.
                    remappings=[
                        ('cmd_vel', stage.input_topic),
                        ('cmd_vel_smoothed', stage.output_topic),
                    ],
                )
            )
            lifecycle_nodes.append(VELOCITY_SMOOTHER)
        elif stage.name == COLLISION_MONITOR:
            actions.append(
                Node(
                    package='nav2_collision_monitor',
                    executable='collision_monitor',
                    name=COLLISION_MONITOR,
                    output='screen',
                    parameters=[
                        _config('collision_monitor.yaml'),
                        # Supplied here so the wiring cannot disagree with the
                        # rest of the chain.
                        {
                            'cmd_vel_in_topic': stage.input_topic,
                            'cmd_vel_out_topic': stage.output_topic,
                        },
                    ],
                )
            )
            lifecycle_nodes.append(COLLISION_MONITOR)

    if lifecycle_nodes:
        # Both Nav2 nodes are lifecycle nodes and bond to a manager. Without
        # one they stay unconfigured and silently pass nothing through.
        actions.append(
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='vla_tracking_lifecycle_manager',
                output='screen',
                parameters=[{
                    'autostart': True,
                    'node_names': lifecycle_nodes,
                }],
            )
        )

    return actions


def generate_launch_description() -> LaunchDescription:
    """Declare the launch arguments and defer node construction."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=_config('tracking.yaml'),
            description='Core parameters for both nodes.',
        ),
        DeclareLaunchArgument(
            'backend',
            default_value='fake',
            description=(
                'Backend entry point name: fake for bring-up, omtrackvla for '
                'the released checkpoint.'
            ),
        ),
        DeclareLaunchArgument(
            'image_topic',
            default_value='/camera/image_raw',
            description='Monocular RGB input.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description=(
                'Follow /clock instead of the wall clock. Required against a '
                'simulator, whose message stamps start at zero: with the wall '
                'clock every frame reads as an epoch old and the staleness '
                'checks drop all of them. Only valid while the simulator runs '
                'at a real-time factor of 1.0; see D021.'
            ),
        ),
        DeclareLaunchArgument(
            'enable_velocity_smoother',
            default_value='false',
            description='Insert the Nav2 velocity smoother into the chain.',
        ),
        DeclareLaunchArgument(
            'enable_collision_monitor',
            default_value='false',
            description=(
                'Insert the Nav2 collision monitor into the chain. Requires a '
                'real sensor on the configured source topic; with no data it '
                'holds the base stopped.'
            ),
        ),
        OpaqueFunction(function=_setup),
    ])
