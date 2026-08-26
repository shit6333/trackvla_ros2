"""
Project-owned contract between the ROS runtime and a VLA model.

Nothing here may import a model library. The runtime depends on this module;
model-specific packages implement it. Timestamps are plain integer nanoseconds
rather than ROS message types so that a backend and its tests can run without
a sourced ROS overlay.
"""

from dataclasses import dataclass
from typing import Any, Mapping, Protocol, runtime_checkable

import numpy as np

#: Number of columns in a waypoint row: forward, lateral, yaw.
WAYPOINT_COLUMNS = 3


class BackendError(RuntimeError):
    """Raised when a backend cannot satisfy a request."""


class BackendNotConfiguredError(BackendError):
    """Raised when a backend is used before configure() succeeded."""


class BackendNotReadyError(BackendError):
    """Raised when infer() is called before a task has been started."""


@dataclass(frozen=True)
class Observation:
    """
    One camera frame offered to a backend.

    :param rgb: HxWx3 uint8 array in RGB channel order.
    :param stamp_ns: Capture time of this frame, in nanoseconds.
    :param instruction: Natural-language description of the target.
    """

    rgb: np.ndarray
    stamp_ns: int
    instruction: str

    def __post_init__(self):
        """Reject malformed frames at construction rather than at inference."""
        if self.rgb.ndim != 3 or self.rgb.shape[2] != 3:
            raise ValueError(
                f'rgb must be HxWx3, got {self.rgb.shape}'
            )
        if self.rgb.dtype != np.uint8:
            raise ValueError(
                f'rgb must be uint8, got {self.rgb.dtype}'
            )
        if self.stamp_ns < 0:
            raise ValueError(f'stamp_ns must be >= 0, got {self.stamp_ns}')


@dataclass(frozen=True)
class Prediction:
    """
    A backend's normalized answer for one observation.

    :param waypoints: (N, 3) float array of x, y, theta in metres and radians.
        Row 0 is the trajectory origin, so executable rows start at index 1.
    :param dt: Seconds between consecutive waypoints. Comes from the backend
        or its checkpoint metadata and is never inferred by a consumer.
    :param frame_id: Robot frame the waypoints are relative to.
    :param stamp_ns: Timestamp of the observation this came from, copied
        through unchanged so a consumer can measure staleness.
    :param valid: False when the result must not be executed.
    :param warming_up: True while the backend is still filling its temporal
        history. Such a prediction is published for visibility but never
        executed, so warming_up implies not valid.
    :param status: Human-readable detail, especially when valid is False.
    """

    waypoints: np.ndarray
    dt: float
    frame_id: str
    stamp_ns: int
    valid: bool
    warming_up: bool
    status: str

    def __post_init__(self):
        """Enforce the invariants every consumer is allowed to rely on."""
        if self.waypoints.ndim != 2 or \
                self.waypoints.shape[1] != WAYPOINT_COLUMNS:
            raise ValueError(
                f'waypoints must be (N, {WAYPOINT_COLUMNS}), '
                f'got {self.waypoints.shape}'
            )
        if self.valid:
            if not np.all(np.isfinite(self.waypoints)):
                raise ValueError('a valid prediction cannot contain NaN or inf')
            if self.waypoints.shape[0] < 2:
                raise ValueError(
                    'a valid prediction needs an origin and at least one '
                    'executable waypoint'
                )
            if not self.dt > 0.0:
                raise ValueError(f'a valid prediction needs dt > 0, got {self.dt}')
            if not self.frame_id:
                raise ValueError('a valid prediction needs a frame_id')
        if self.warming_up and self.valid:
            raise ValueError('a warming-up prediction must not be valid')


@runtime_checkable
class VLABackend(Protocol):
    """
    Lifecycle a model must implement to be driven by vla_inference_node.

    The backend owns whatever temporal state its model needs. The node keeps
    only the most recent frame and calls reset() on every task change, because
    a model such as OmTrackVLA stores encoded tokens rather than raw images
    and must not be asked to re-encode a window on every step.
    """

    @property
    def name(self) -> str:
        """Return the identifier recorded in published trajectories."""
        ...

    def configure(self, config: Mapping[str, Any]) -> None:
        """Load weights and allocate resources. Must be idempotent."""
        ...

    def reset(self, instruction: str) -> None:
        """Start a new task, discarding all state from the previous one."""
        ...

    def infer(self, observation: Observation) -> Prediction:
        """Consume one frame and return the current trajectory estimate."""
        ...

    def shutdown(self) -> None:
        """Release resources. Must tolerate being called more than once."""
        ...
