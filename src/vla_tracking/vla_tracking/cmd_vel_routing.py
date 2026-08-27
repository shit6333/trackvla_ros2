"""
Wiring of the optional Nav2 command-processing chain.

Getting this wrong is dangerous in a specific way: if two nodes end up
publishing the final command topic, the base acts on whichever message
arrives last and its behaviour becomes non-deterministic. The routing is
therefore computed by one pure function that the launch file consumes and the
tests check exhaustively, rather than by conditional remappings scattered
through a launch description.
"""

from dataclasses import dataclass
from typing import Tuple

#: The topic the base actually listens to. Exactly one component may publish it.
FINAL_TOPIC = 'cmd_vel'

#: What the executor publishes when anything is downstream of it.
RAW_TOPIC = 'cmd_vel_raw'

#: Ordered chain of optional stages and the topic each publishes when it is not
#: the last enabled stage.
VELOCITY_SMOOTHER = 'velocity_smoother'
COLLISION_MONITOR = 'collision_monitor'

_CHAIN_SPEC: Tuple[Tuple[str, str], ...] = (
    (VELOCITY_SMOOTHER, 'cmd_vel_smoothed'),
    (COLLISION_MONITOR, 'cmd_vel_checked'),
)


@dataclass(frozen=True)
class Stage:
    """One enabled node in the command chain."""

    name: str
    input_topic: str
    output_topic: str


@dataclass(frozen=True)
class ChainPlan:
    """The complete wiring for one combination of enabled stages."""

    executor_output: str
    stages: Tuple[Stage, ...]
    final_publisher: str

    @property
    def enabled_names(self) -> Tuple[str, ...]:
        """Return the names of the enabled stages, in chain order."""
        return tuple(stage.name for stage in self.stages)


def plan_cmd_vel_chain(
    enable_velocity_smoother: bool = False,
    enable_collision_monitor: bool = False,
) -> ChainPlan:
    """
    Compute the topic wiring for the requested chain.

    With nothing enabled the executor publishes the final topic directly, so a
    raw experiment is genuinely bypassing Nav2 rather than passing through
    stages configured to do nothing (see D004). With anything enabled the
    executor moves to the raw topic and the last enabled stage takes over the
    final one.
    """
    requested = {
        VELOCITY_SMOOTHER: enable_velocity_smoother,
        COLLISION_MONITOR: enable_collision_monitor,
    }
    enabled = [
        (name, intermediate)
        for name, intermediate in _CHAIN_SPEC
        if requested[name]
    ]

    if not enabled:
        return ChainPlan(
            executor_output=FINAL_TOPIC,
            stages=(),
            final_publisher='trajectory_executor_node',
        )

    stages = []
    upstream = RAW_TOPIC
    for index, (name, intermediate) in enumerate(enabled):
        is_last = index == len(enabled) - 1
        output = FINAL_TOPIC if is_last else intermediate
        stages.append(
            Stage(name=name, input_topic=upstream, output_topic=output)
        )
        upstream = output

    return ChainPlan(
        executor_output=RAW_TOPIC,
        stages=tuple(stages),
        final_publisher=stages[-1].name,
    )


def publishers_by_topic(plan: ChainPlan) -> dict:
    """
    Map every topic in a plan to the components that publish it.

    Exposed so the invariant can be asserted directly instead of being
    re-derived by each test.
    """
    publishers: dict = {}
    publishers.setdefault(plan.executor_output, []).append(
        'trajectory_executor_node'
    )
    for stage in plan.stages:
        publishers.setdefault(stage.output_topic, []).append(stage.name)
    return publishers
