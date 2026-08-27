"""
GPU-backed smoke test of the whole pipeline against the real checkpoint.

Everything else runs on the fake backend, so this is the only automated check
that the released weights work through the ROS graph rather than only through
the backend contract. It skips cleanly without CUDA or a checkpoint.
"""

import os
import pathlib

import pytest
from rclpy.parameter import Parameter

torch = pytest.importorskip('torch')

from vla_tracking.inference_node import VlaInferenceNode  # noqa: E402
from vla_tracking.testing import (  # noqa: E402
    FakeCamera,
    PipelineObserver,
    SpinningGraph,
    wait_until,
)
from vla_tracking.trajectory_executor_node import (  # noqa: E402
    TrajectoryExecutorNode,
)

INSTRUCTION = 'follow the person in the red shirt'


def _available() -> bool:
    """Report whether CUDA, the upstream source, and weights are all present."""
    if not torch.cuda.is_available():
        return False
    model_dir = os.environ.get('HF_MODEL_DIR')
    source = os.environ.get('OMTRACKVLA_SRC')
    if not model_dir or not source:
        return False
    return (
        (pathlib.Path(model_dir) / 'config.json').is_file()
        and (pathlib.Path(source) / 'model.py').is_file()
    )


requires_model = pytest.mark.skipif(
    not _available(),
    reason='needs CUDA, OMTRACKVLA_SRC and a checkpoint at HF_MODEL_DIR',
)


@requires_model
def test_real_weights_drive_the_pipeline(capsys):
    """
    Check the released checkpoint produces motion through the ROS graph.

    A short history is used so the warm-up does not dominate the test; the
    full 31-frame history is exercised by the adapter's own tests.
    """
    graph = SpinningGraph()
    camera = None
    try:
        context = graph.context
        graph.add(VlaInferenceNode(
            context=context,
            parameter_overrides=[
                Parameter('backend', value='omtrackvla'),
                Parameter('history_length', value=4),
                Parameter('inference_rate', value=10.0),
                Parameter('require_cuda', value=True),
            ],
        ))
        graph.add(TrajectoryExecutorNode(
            context=context,
            parameter_overrides=[
                Parameter('command_rate', value=20.0),
                Parameter('trajectory_timeout', value=0.5),
            ],
        ))
        camera = graph.add(FakeCamera(context, rate=15.0))
        observer = graph.add(PipelineObserver(context))

        graph.start()
        camera.start()

        observer.start_task(INSTRUCTION, timeout=120.0)

        wait_until(
            lambda: len(observer.valid_trajectories()) >= 2,
            timeout=120.0,
            description='the real model to warm up and publish',
        )
        wait_until(
            lambda: len(observer.moving_commands()) >= 2,
            timeout=60.0,
            description='the executor to command motion from real predictions',
        )

        trajectory = observer.valid_trajectories()[-1]
        assert trajectory.backend_name == 'omtrackvla'
        assert len(trajectory.waypoints) == 8
        assert trajectory.dt == pytest.approx(0.1)

        durations = [
            status.last_inference_duration
            for status in observer.statuses
            if status.last_inference_duration > 0.0
        ]
        assert durations, 'the status never reported an inference duration'
        with capsys.disabled():
            print(
                f'\n[omtrackvla] in-pipeline inference '
                f'{max(durations) * 1000.0:.0f} ms worst case over '
                f'{len(durations)} samples'
            )
    finally:
        if camera is not None:
            camera.stop()
        graph.shutdown()
