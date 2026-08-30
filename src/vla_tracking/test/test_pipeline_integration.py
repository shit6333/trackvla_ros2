"""
Whole-pipeline tests: instruction in, velocity commands out.

The node-level tests each drive one node with synthetic input from the other
side. This file runs both nodes together over a real graph, which is the only
place the handover between them is exercised: the trajectory the inference
node publishes is the one the executor validates, converts, and clamps.

The fake backend is used, so these need no GPU and no weights.
"""

import pytest
from rclpy.parameter import Parameter

from vla_tracking.inference_node import VlaInferenceNode
from vla_tracking.testing import (
    FakeCamera,
    PipelineObserver,
    SpinningGraph,
    wait_until,
)
from vla_tracking.trajectory_executor_node import TrajectoryExecutorNode

from vla_tracking_interfaces.msg import VlaStatus

INSTRUCTION = 'follow the person in the red shirt'
HISTORY_LENGTH = 5


class Pipeline:
    """Both runtime nodes, a camera, and an observer on one graph."""

    def __init__(self, backend='fake', executor_overrides=None):
        """Build and start the whole pipeline."""
        self.graph = SpinningGraph()
        context = self.graph.context

        self.inference = self.graph.add(VlaInferenceNode(
            context=context,
            parameter_overrides=[
                Parameter('backend', value=backend),
                Parameter('history_length', value=HISTORY_LENGTH),
                Parameter('inference_rate', value=20.0),
                Parameter('require_cuda', value=False),
            ],
        ))
        self.executor_node = self.graph.add(TrajectoryExecutorNode(
            context=context,
            parameter_overrides=list(executor_overrides or []) + [
                Parameter('command_rate', value=50.0),
                Parameter('trajectory_timeout', value=0.5),
            ],
        ))
        self.camera = self.graph.add(FakeCamera(context, rate=30.0))
        self.observer = self.graph.add(PipelineObserver(context))

        self.graph.start()
        self.camera.start()

    def shutdown(self):
        """Stop the camera and tear the graph down."""
        self.camera.stop()
        self.graph.shutdown()


@pytest.fixture
def pipeline():
    """Provide a running pipeline with guaranteed teardown."""
    running = Pipeline()
    try:
        yield running
    finally:
        running.shutdown()


def test_nothing_moves_before_a_goal(pipeline):
    """Check frames alone never move the base."""
    wait_until(
        lambda: pipeline.camera.frames_published > 10,
        timeout=10.0,
        description='the camera to produce frames',
    )
    assert pipeline.observer.trajectories == []
    assert pipeline.observer.moving_commands() == []


def test_an_instruction_reaches_the_wheels(pipeline):
    """
    Check the whole chain, from a spoken instruction to a velocity command.

    This is the handover the node-level tests cannot cover: the trajectory
    published by inference is the one the executor admits, converts, clamps,
    and turns into motion.
    """
    goal = pipeline.observer.start_task(INSTRUCTION)

    wait_until(
        lambda: len(pipeline.observer.valid_trajectories()) >= 2,
        timeout=25.0,
        description='the backend to finish warming up and publish',
    )
    wait_until(
        lambda: len(pipeline.observer.moving_commands()) >= 2,
        timeout=15.0,
        description='the executor to command motion',
    )

    trajectory = pipeline.observer.valid_trajectories()[-1]
    assert trajectory.backend_name == 'fake'
    assert trajectory.header.frame_id == 'base_link'
    assert len(trajectory.waypoints) == 8

    for command in pipeline.observer.moving_commands():
        assert abs(command.linear.x) <= 0.2 + 1e-9
        assert abs(command.angular.z) <= 0.5 + 1e-9

    goal.cancel_goal_async()


def test_the_base_is_still_while_the_backend_warms_up(pipeline):
    """
    Check the warm-up policy holds across the node boundary.

    The inference node publishes warm-up predictions so an operator can watch
    progress, and the executor must refuse every one of them. A regression
    here would drive the robot from a padded history.
    """
    pipeline.observer.start_task(INSTRUCTION)

    wait_until(
        lambda: len(pipeline.observer.trajectories) >= 1,
        timeout=20.0,
        description='the first prediction',
    )
    warm_up = list(pipeline.observer.trajectories[:HISTORY_LENGTH - 1])
    assert warm_up, 'no warm-up predictions were observed'
    assert all(not msg.valid for msg in warm_up)
    assert all('warming up' in msg.status for msg in warm_up)


def test_cancelling_stops_the_base(pipeline):
    """Check a cancelled task leaves no command latched."""
    goal = pipeline.observer.start_task(INSTRUCTION)
    wait_until(
        lambda: len(pipeline.observer.moving_commands()) >= 2,
        timeout=25.0,
        description='the base to start moving',
    )

    goal.cancel_goal_async()

    wait_until(
        lambda: any(
            status.task_state in (VlaStatus.STOPPED, VlaStatus.IDLE)
            for status in pipeline.observer.statuses[-5:]
        ),
        timeout=15.0,
        description='the task to report that it stopped',
    )

    # A plan holds its last segment until trajectory_timeout, so the zeros
    # only start 0.5 s after the final trajectory. At command_rate 50 Hz that
    # is 25 cycles; wait for twice that so the tail below cannot straddle the
    # moment the timeout fires.
    settled = len(pipeline.observer.commands)
    wait_until(
        lambda: len(pipeline.observer.commands) > settled + 50,
        timeout=10.0,
        description='further command cycles after cancellation',
    )
    tail = pipeline.observer.commands[-10:]
    assert all(
        command.linear.x == 0.0
        and command.linear.y == 0.0
        and command.angular.z == 0.0
        for command in tail
    ), 'a command stayed latched after the task was cancelled'
