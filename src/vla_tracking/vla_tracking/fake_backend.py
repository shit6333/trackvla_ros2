"""
Deterministic stand-in backend that needs no model weights.

It exists so node and integration tests can exercise the whole ROS path in
seconds. It imitates the parts of a real backend's behaviour that the runtime
depends on: a temporal history that must be filled before results become
valid, and a reset that discards it.
"""

from collections import deque
from typing import Any, Mapping

import numpy as np

from vla_tracking.backend_interface import (
    BackendNotConfiguredError,
    BackendNotReadyError,
    Observation,
    Prediction,
)

#: Matches the OmTrackVLA checkpoint so tests exercise realistic shapes.
DEFAULT_HISTORY_LENGTH = 31
DEFAULT_WAYPOINT_COUNT = 8
DEFAULT_DT = 0.1
DEFAULT_FRAME_ID = 'base_link'


class FakeBackend:
    """A backend whose output is a pure function of its inputs."""

    def __init__(self):
        """Create an unconfigured backend."""
        self._configured = False
        self._instruction = None
        self._history = deque(maxlen=DEFAULT_HISTORY_LENGTH)
        self._history_length = DEFAULT_HISTORY_LENGTH
        self._waypoint_count = DEFAULT_WAYPOINT_COUNT
        self._dt = DEFAULT_DT
        self._frame_id = DEFAULT_FRAME_ID

    @property
    def name(self) -> str:
        """Return the identifier recorded in published trajectories."""
        return 'fake'

    def configure(self, config: Mapping[str, Any]) -> None:
        """Apply optional overrides for history length, dt, and frame."""
        self._history_length = int(
            config.get('history_length', DEFAULT_HISTORY_LENGTH)
        )
        if self._history_length < 1:
            raise ValueError('history_length must be at least 1')
        self._waypoint_count = int(
            config.get('waypoint_count', DEFAULT_WAYPOINT_COUNT)
        )
        self._dt = float(config.get('dt', DEFAULT_DT))
        self._frame_id = str(config.get('frame_id', DEFAULT_FRAME_ID))
        self._history = deque(maxlen=self._history_length)
        self._configured = True

    def reset(self, instruction: str) -> None:
        """Start a new task and drop the previous task's history."""
        if not self._configured:
            raise BackendNotConfiguredError(
                'configure() must succeed before reset()'
            )
        self._instruction = instruction
        self._history.clear()

    def infer(self, observation: Observation) -> Prediction:
        """Return a trajectory derived deterministically from the history."""
        if not self._configured:
            raise BackendNotConfiguredError(
                'configure() must succeed before infer()'
            )
        if self._instruction is None:
            raise BackendNotReadyError('reset() must be called before infer()')

        self._history.append(float(observation.rgb.mean()))
        warming_up = len(self._history) < self._history_length

        # A cheap, stable function of the whole history, so that appending a
        # frame changes the result and clearing it restores the earlier one.
        seed = int(abs(sum(self._history)) * 1000.0) % (2 ** 31)
        rng = np.random.default_rng(seed)
        waypoints = np.zeros((self._waypoint_count, 3), dtype=np.float64)
        waypoints[1:] = rng.normal(scale=0.05, size=(self._waypoint_count - 1, 3))

        return Prediction(
            waypoints=waypoints,
            dt=self._dt,
            frame_id=self._frame_id,
            stamp_ns=observation.stamp_ns,
            valid=not warming_up,
            warming_up=warming_up,
            status=(
                f'warming up, {len(self._history)}/{self._history_length} '
                f'frames' if warming_up else 'ok'
            ),
        )

    def describe(self):
        """Report provenance for the startup health log."""
        return {
            'backend': 'fake',
            'weights': 'none, output is a function of the input',
            'history_length': str(self._history_length),
            'dt': f'{self._dt:.3f} s',
        }

    def shutdown(self) -> None:
        """Drop all state. Safe to call repeatedly."""
        self._history.clear()
        self._instruction = None
        self._configured = False
