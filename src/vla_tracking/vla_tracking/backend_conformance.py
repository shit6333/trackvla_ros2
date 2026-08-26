"""
Reusable conformance suite for backend implementations.

Both the fake backend and every model adapter are checked against exactly the
same assertions, which is what makes "the backends satisfy one contract" a
verified statement rather than an intention. A new adapter should call
check_conformance from its own test package.
"""

from typing import Any, Callable, Mapping

import numpy as np

from vla_tracking.backend_interface import (
    BackendNotReadyError,
    Observation,
    VLABackend,
)


def make_frame(seed: int, shape=(120, 160)) -> np.ndarray:
    """Build a deterministic RGB frame for a given seed."""
    rng = np.random.default_rng(seed)
    return rng.integers(0, 255, (*shape, 3), dtype=np.uint8)


def check_conformance(
    factory: Callable[[], VLABackend],
    config: Mapping[str, Any],
    frame_shape=(120, 160),
) -> dict:
    """
    Assert that a backend honours the contract, and report what was observed.

    :param factory: Callable returning a fresh, unconfigured backend.
    :param config: Configuration passed to configure().
    :param frame_shape: Height and width of the synthetic frames used.
    :returns: Observations a caller may want to log, such as the waypoint
        count and the number of frames needed before results became valid.
    """
    report = {}

    _check_infer_before_reset_is_refused(factory, config, frame_shape)
    report.update(_check_first_prediction(factory, config, frame_shape))
    report.update(_check_reset_clears_history(factory, config, frame_shape))

    return report


def _check_infer_before_reset_is_refused(factory, config, frame_shape):
    """Check a configured but un-started backend refuses to infer."""
    backend = factory()
    backend.configure(config)
    try:
        observation = Observation(
            rgb=make_frame(0, frame_shape),
            stamp_ns=1_000_000_000,
            instruction='follow the person in the red shirt',
        )
        try:
            backend.infer(observation)
        except BackendNotReadyError:
            pass
        else:
            raise AssertionError(
                'infer() before reset() must raise BackendNotReadyError'
            )
    finally:
        backend.shutdown()


def _check_first_prediction(factory, config, frame_shape):
    """Check the first prediction is well formed and carries metadata."""
    backend = factory()
    backend.configure(config)
    try:
        backend.reset('follow the person in the red shirt')
        stamp = 1_234_567_890
        observation = Observation(
            rgb=make_frame(1, frame_shape),
            stamp_ns=stamp,
            instruction='follow the person in the red shirt',
        )
        prediction = backend.infer(observation)

        assert prediction.waypoints.ndim == 2, 'waypoints must be 2-D'
        assert prediction.waypoints.shape[1] == 3, 'waypoints must have 3 columns'
        assert np.all(np.isfinite(prediction.waypoints)), 'waypoints must be finite'
        assert prediction.dt > 0.0, 'dt must be positive and explicit'
        assert prediction.frame_id, 'frame_id must be set'
        assert prediction.stamp_ns == stamp, \
            'the prediction must carry the observation stamp unchanged'
        assert backend.name, 'name must identify the backend'

        return {
            'waypoint_count': int(prediction.waypoints.shape[0]),
            'dt': float(prediction.dt),
            'frame_id': prediction.frame_id,
            'first_prediction_warming_up': bool(prediction.warming_up),
        }
    finally:
        backend.shutdown()


def _check_reset_clears_history(factory, config, frame_shape):
    """
    Check that appending a frame changes the result and reset undoes it.

    This is the property the node relies on when a new instruction preempts an
    active task: nothing from the previous target may survive into the next
    prediction.
    """
    backend = factory()
    backend.configure(config)
    try:
        instruction = 'follow the person in the red shirt'
        frame_a = make_frame(1, frame_shape)
        frame_b = make_frame(2, frame_shape)

        backend.reset(instruction)
        first = backend.infer(
            Observation(rgb=frame_a, stamp_ns=1, instruction=instruction)
        )
        second = backend.infer(
            Observation(rgb=frame_b, stamp_ns=2, instruction=instruction)
        )
        assert not np.array_equal(first.waypoints, second.waypoints), \
            'a second frame must change the prediction'

        backend.reset(instruction)
        after_reset = backend.infer(
            Observation(rgb=frame_a, stamp_ns=3, instruction=instruction)
        )
        assert np.array_equal(first.waypoints, after_reset.waypoints), \
            'reset() must restore the first prediction exactly'
        assert after_reset.stamp_ns == 3, \
            'the prediction must carry the new observation stamp'

        return {'reset_restores_first_prediction': True}
    finally:
        backend.shutdown()
