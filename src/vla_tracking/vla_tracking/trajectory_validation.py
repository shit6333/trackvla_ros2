"""
Admission checks for trajectories arriving at the executor.

This is the last barrier between a black-box model and the wheels, so every
rejection carries a reason: an executor that silently discards input is
indistinguishable from one whose upstream has died.
"""

from dataclasses import dataclass
import math

NANOSECONDS_PER_SECOND = 1_000_000_000

#: A trajectory needs an origin and at least one point to move towards.
MINIMUM_WAYPOINTS = 2


@dataclass(frozen=True)
class ValidationResult:
    """Outcome of checking one trajectory."""

    accepted: bool
    reason: str

    def __bool__(self) -> bool:
        """Allow a result to be used directly in a condition."""
        return self.accepted


def _accept() -> ValidationResult:
    """Build an accepting result."""
    return ValidationResult(accepted=True, reason='')


def _reject(reason: str) -> ValidationResult:
    """Build a rejecting result carrying a diagnosable reason."""
    return ValidationResult(accepted=False, reason=reason)


def validate_trajectory(
    message,
    *,
    now_ns: int,
    base_frame: str = '',
    require_valid: bool = True,
    max_age_seconds: float = 0.0,
) -> ValidationResult:
    """
    Decide whether a trajectory may be executed.

    :param now_ns: Current time, for the staleness check.
    :param base_frame: Frame the executor commands in. Empty disables the
        check, for a deployment whose frames are remapped elsewhere.
    :param require_valid: Reject trajectories the backend itself marked
        invalid, which includes every warm-up prediction.
    :param max_age_seconds: Reject a trajectory whose observation is older
        than this. Zero disables the check, which is appropriate when the
        camera's clock is not synchronised with the executor's.
    """
    if require_valid and not message.valid:
        return _reject(f'backend marked it invalid: {message.status!r}')

    count = len(message.waypoints)
    if count < MINIMUM_WAYPOINTS:
        return _reject(
            f'{count} waypoints, need at least {MINIMUM_WAYPOINTS}'
        )

    if not math.isfinite(message.dt) or message.dt <= 0.0:
        return _reject(f'dt is {message.dt}, must be finite and positive')

    if base_frame and message.header.frame_id != base_frame:
        return _reject(
            f'frame_id is {message.header.frame_id!r}, expected {base_frame!r}'
        )

    for index, waypoint in enumerate(message.waypoints):
        for axis, value in (
            ('x', waypoint.x), ('y', waypoint.y), ('theta', waypoint.theta)
        ):
            if not math.isfinite(value):
                return _reject(
                    f'waypoint {index} has non-finite {axis} ({value})'
                )

    if max_age_seconds > 0.0:
        stamp_ns = (
            message.header.stamp.sec * NANOSECONDS_PER_SECOND
            + message.header.stamp.nanosec
        )
        if stamp_ns == 0:
            return _reject('header stamp is unset')
        age_seconds = (now_ns - stamp_ns) / NANOSECONDS_PER_SECOND
        if age_seconds > max_age_seconds:
            return _reject(
                f'observation is {age_seconds * 1000.0:.0f} ms old, '
                f'limit is {max_age_seconds * 1000.0:.0f} ms'
            )

    return _accept()
