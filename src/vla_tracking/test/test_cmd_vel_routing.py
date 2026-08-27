"""
Exhaustive checks of the command-chain wiring.

There are only four combinations, so all four are checked directly rather
than sampled. The property that matters is that exactly one component
publishes the topic the base listens to.
"""

import itertools

import pytest

from vla_tracking.cmd_vel_routing import (
    COLLISION_MONITOR,
    FINAL_TOPIC,
    plan_cmd_vel_chain,
    publishers_by_topic,
    RAW_TOPIC,
    VELOCITY_SMOOTHER,
)

ALL_COMBINATIONS = list(itertools.product([False, True], repeat=2))


@pytest.mark.parametrize('smoother,monitor', ALL_COMBINATIONS)
def test_exactly_one_component_publishes_the_final_topic(smoother, monitor):
    """Check the base can never receive commands from two publishers."""
    plan = plan_cmd_vel_chain(smoother, monitor)
    publishers = publishers_by_topic(plan)
    assert publishers[FINAL_TOPIC] == [plan.final_publisher], (
        f'smoother={smoother} monitor={monitor}: {FINAL_TOPIC} is published '
        f'by {publishers[FINAL_TOPIC]}'
    )


@pytest.mark.parametrize('smoother,monitor', ALL_COMBINATIONS)
def test_no_topic_has_two_publishers(smoother, monitor):
    """Check no intermediate topic is contended either."""
    plan = plan_cmd_vel_chain(smoother, monitor)
    for topic, names in publishers_by_topic(plan).items():
        assert len(names) == 1, f'{topic} is published by {names}'


@pytest.mark.parametrize('smoother,monitor', ALL_COMBINATIONS)
def test_the_chain_is_connected(smoother, monitor):
    """Check every stage consumes what the stage before it produces."""
    plan = plan_cmd_vel_chain(smoother, monitor)
    upstream = plan.executor_output
    for stage in plan.stages:
        assert stage.input_topic == upstream, (
            f'{stage.name} reads {stage.input_topic} but the previous stage '
            f'writes {upstream}'
        )
        upstream = stage.output_topic
    assert upstream == FINAL_TOPIC


def test_raw_mode_bypasses_the_chain_entirely():
    """
    Check a disabled chain is absent, not neutralised.

    D004 requires that a raw experiment bypass Nav2 completely rather than
    run stages configured with no-op thresholds, so that a measurement cannot
    be quietly shaped by a smoother nobody remembered was running.
    """
    plan = plan_cmd_vel_chain(False, False)
    assert plan.stages == ()
    assert plan.executor_output == FINAL_TOPIC
    assert plan.final_publisher == 'trajectory_executor_node'


def test_smoother_only():
    """Check the smoother alone takes over the final topic."""
    plan = plan_cmd_vel_chain(True, False)
    assert plan.executor_output == RAW_TOPIC
    assert plan.enabled_names == (VELOCITY_SMOOTHER,)
    assert plan.stages[0].input_topic == RAW_TOPIC
    assert plan.stages[0].output_topic == FINAL_TOPIC


def test_monitor_only():
    """Check the monitor alone reads the raw topic, not a smoothed one."""
    plan = plan_cmd_vel_chain(False, True)
    assert plan.enabled_names == (COLLISION_MONITOR,)
    assert plan.stages[0].input_topic == RAW_TOPIC
    assert plan.stages[0].output_topic == FINAL_TOPIC


def test_both_enabled_keeps_the_monitor_last():
    """
    Check the monitor is the final stage when both are enabled.

    Order is not arbitrary: the collision monitor must judge the command that
    will actually be sent, so smoothing has to happen before it, never after.
    """
    plan = plan_cmd_vel_chain(True, True)
    assert plan.enabled_names == (VELOCITY_SMOOTHER, COLLISION_MONITOR)
    assert plan.stages[0].input_topic == RAW_TOPIC
    assert plan.stages[0].output_topic == plan.stages[1].input_topic
    assert plan.stages[1].output_topic == FINAL_TOPIC
    assert plan.final_publisher == COLLISION_MONITOR
