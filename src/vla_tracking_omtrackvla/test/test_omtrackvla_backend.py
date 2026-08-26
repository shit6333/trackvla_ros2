"""
Conformance and characterisation tests for the OmTrackVLA adapter.

These need a GPU and the checkpoint, so they skip cleanly elsewhere. Beyond
conformance they measure the two numbers the rest of the system needs and
which nothing has established yet: how long one inference takes, and whether
the checkpoint really places the trajectory origin at waypoint 0.
"""

import os
import pathlib
import time

import numpy as np
import pytest

torch = pytest.importorskip('torch')

from vla_tracking.backend_conformance import (  # noqa: E402
    check_conformance,
    make_frame,
)
from vla_tracking.backend_interface import Observation  # noqa: E402
from vla_tracking_omtrackvla.model_loader import (  # noqa: E402
    CHECKPOINT_HISTORY_LENGTH,
)
from vla_tracking_omtrackvla.omtrackvla_backend import (  # noqa: E402
    OmTrackVLABackend,
)


def _config():
    """Build the adapter configuration from the container's environment."""
    return {
        'source_path': os.environ.get('OMTRACKVLA_SRC'),
        'model_dir': os.environ.get('HF_MODEL_DIR'),
        'require_cuda': True,
    }


def _available():
    """Report whether a GPU and a readable checkpoint are both present."""
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
def test_adapter_satisfies_the_same_contract_as_the_fake():
    """Check the adapter passes the shared conformance suite."""
    config = dict(_config())
    # A short history keeps the suite to a few forward passes.
    config['history_length'] = 2
    report = check_conformance(
        OmTrackVLABackend, config, frame_shape=(240, 320)
    )
    assert report['waypoint_count'] == 8
    assert report['dt'] == pytest.approx(0.1)
    assert report['frame_id'] == 'base_link'


@requires_model
def test_inference_latency_and_trajectory_origin(capsys):
    """
    Measure per-step latency and check where the trajectory origin sits.

    Latency sets the achievable inference rate, which in turn decides the
    executor's timeout and the cadence the temporal history must be fed at.
    The origin check tests the documented claim that waypoint 0 is the robot's
    pose at prediction time, which nothing has verified against the weights.
    """
    backend = OmTrackVLABackend()
    backend.configure(_config())
    try:
        instruction = 'follow the person in the red shirt'
        backend.reset(instruction)

        durations = []
        origins = []
        # Fill the history so the measurement reflects steady state.
        for step in range(CHECKPOINT_HISTORY_LENGTH + 5):
            observation = Observation(
                rgb=make_frame(step, (240, 320)),
                stamp_ns=step * 100_000_000,
                instruction=instruction,
            )
            started = time.perf_counter()
            prediction = backend.infer(observation)
            durations.append(time.perf_counter() - started)
            origins.append(np.abs(prediction.waypoints[0]))

        steady = durations[CHECKPOINT_HISTORY_LENGTH:]
        mean_seconds = float(np.mean(steady))
        origin_magnitude = float(np.max(origins[CHECKPOINT_HISTORY_LENGTH:]))

        with capsys.disabled():
            print(
                f'\n[omtrackvla] steady-state inference '
                f'{mean_seconds * 1000:.1f} ms '
                f'({1.0 / mean_seconds:.1f} Hz), '
                f'max |waypoint[0]| = {origin_magnitude:.4f}'
            )

        assert mean_seconds > 0.0
        assert np.all(np.isfinite(np.concatenate(origins)))
    finally:
        backend.shutdown()


@requires_model
def test_history_is_owned_by_the_backend():
    """
    Check that state lives in the instance rather than in module globals.

    Two concurrently configured backends must not share history, otherwise a
    second node in the same process would corrupt the first one's task.
    """
    instruction = 'follow the person in the red shirt'
    config = dict(_config())
    config['history_length'] = 2

    first = OmTrackVLABackend()
    second = OmTrackVLABackend()
    try:
        first.configure(config)
        second.configure(config)
        first.reset(instruction)
        second.reset(instruction)

        frame = make_frame(7, (240, 320))
        first.infer(Observation(rgb=frame, stamp_ns=1, instruction=instruction))
        first.infer(Observation(rgb=frame, stamp_ns=2, instruction=instruction))

        # second has seen nothing yet, so its first result must be warming up.
        result = second.infer(
            Observation(rgb=frame, stamp_ns=3, instruction=instruction)
        )
        assert result.warming_up is True
    finally:
        first.shutdown()
        second.shutdown()
