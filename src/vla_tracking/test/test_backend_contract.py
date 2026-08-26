"""Contract tests for the backend interface and the fake backend."""

import numpy as np
import pytest

from vla_tracking.backend_conformance import check_conformance, make_frame
from vla_tracking.backend_interface import (
    BackendNotConfiguredError,
    Observation,
    Prediction,
    VLABackend,
)
from vla_tracking.fake_backend import FakeBackend


def test_fake_backend_satisfies_the_contract():
    """Check the fake backend passes the same suite adapters must pass."""
    report = check_conformance(FakeBackend, {'history_length': 3})
    assert report['waypoint_count'] == 8
    assert report['dt'] == pytest.approx(0.1)
    assert report['frame_id'] == 'base_link'


def test_fake_backend_is_a_vla_backend():
    """Check the protocol accepts a conforming implementation."""
    assert isinstance(FakeBackend(), VLABackend)


def test_use_before_configure_is_refused():
    """Check an unconfigured backend refuses work instead of guessing."""
    backend = FakeBackend()
    with pytest.raises(BackendNotConfiguredError):
        backend.reset('follow the person')


def test_warm_up_predictions_are_published_but_not_executable():
    """
    Check the agreed warm-up policy.

    While the history is short the backend still returns a prediction so the
    node can report progress, but marks it invalid so nothing drives the base
    from a padded history.
    """
    backend = FakeBackend()
    backend.configure({'history_length': 3})
    backend.reset('follow the person')

    states = []
    for step in range(4):
        prediction = backend.infer(
            Observation(
                rgb=make_frame(step),
                stamp_ns=step + 1,
                instruction='follow the person',
            )
        )
        states.append((prediction.warming_up, prediction.valid))

    assert states[0] == (True, False)
    assert states[1] == (True, False)
    assert states[2] == (False, True)
    assert states[3] == (False, True)


def test_observation_rejects_malformed_frames():
    """Check bad imagery fails at construction, not deep inside a model."""
    with pytest.raises(ValueError):
        Observation(
            rgb=np.zeros((10, 10), dtype=np.uint8),
            stamp_ns=0,
            instruction='x',
        )
    with pytest.raises(ValueError):
        Observation(
            rgb=np.zeros((10, 10, 3), dtype=np.float32),
            stamp_ns=0,
            instruction='x',
        )


def test_prediction_rejects_contradictory_states():
    """Check a prediction cannot be both warming up and executable."""
    waypoints = np.zeros((8, 3), dtype=np.float64)
    with pytest.raises(ValueError):
        Prediction(
            waypoints=waypoints,
            dt=0.1,
            frame_id='base_link',
            stamp_ns=0,
            valid=True,
            warming_up=True,
            status='',
        )


def test_prediction_rejects_non_finite_when_valid():
    """Check a valid prediction can never carry NaN into the executor."""
    waypoints = np.zeros((8, 3), dtype=np.float64)
    waypoints[2, 0] = np.nan
    with pytest.raises(ValueError):
        Prediction(
            waypoints=waypoints,
            dt=0.1,
            frame_id='base_link',
            stamp_ns=0,
            valid=True,
            warming_up=False,
            status='',
        )


def test_invalid_prediction_may_carry_non_finite_values():
    """Check a failed inference can still be reported for debugging."""
    waypoints = np.full((8, 3), np.nan, dtype=np.float64)
    prediction = Prediction(
        waypoints=waypoints,
        dt=0.1,
        frame_id='base_link',
        stamp_ns=0,
        valid=False,
        warming_up=False,
        status='planner failed',
    )
    assert prediction.valid is False
