"""Tests for the geometry that ultimately turns wheels."""

import math

import pytest

from vla_tracking.trajectory_conversion import (
    clamp_segment,
    trajectory_to_segments,
    VelocitySegment,
    wrap_angle,
)


def test_straight_line_becomes_forward_velocity():
    """Check a purely forward step maps to forward speed alone."""
    segments = trajectory_to_segments(
        [(0.0, 0.0, 0.0), (0.05, 0.0, 0.0)], dt=0.1, max_segments=1
    )
    assert len(segments) == 1
    assert segments[0].linear_x == pytest.approx(0.5)
    assert segments[0].linear_y == pytest.approx(0.0)
    assert segments[0].angular_z == pytest.approx(0.0)
    assert segments[0].duration == pytest.approx(0.1)


def test_pure_rotation_becomes_angular_velocity():
    """Check turning in place produces yaw rate and no translation."""
    segments = trajectory_to_segments(
        [(0.0, 0.0, 0.0), (0.0, 0.0, 0.1)], dt=0.1, max_segments=1
    )
    assert segments[0].linear_x == pytest.approx(0.0)
    assert segments[0].angular_z == pytest.approx(1.0)


def test_later_segments_are_expressed_in_the_earlier_pose_frame():
    """
    Check a curving trajectory is rotated into each segment's own frame.

    After the first segment the robot has already turned, so the raw
    difference in the prediction frame is not what it should be commanded.
    Here the robot faces +90 degrees and the path then steps along world +y,
    which is straight ahead from the robot's point of view.
    """
    waypoints = [
        (0.0, 0.0, 0.0),
        (0.0, 0.0, math.pi / 2.0),
        (0.0, 0.05, math.pi / 2.0),
    ]
    segments = trajectory_to_segments(waypoints, dt=0.1, max_segments=2)
    assert len(segments) == 2
    second = segments[1]
    assert second.linear_x == pytest.approx(0.5)
    assert second.linear_y == pytest.approx(0.0, abs=1e-9)
    assert second.angular_z == pytest.approx(0.0)


def test_max_segments_bounds_the_output():
    """Check the executor never gets more segments than it asked for."""
    waypoints = [(float(i) * 0.01, 0.0, 0.0) for i in range(8)]
    assert len(trajectory_to_segments(waypoints, 0.1, 1)) == 1
    assert len(trajectory_to_segments(waypoints, 0.1, 3)) == 3
    # A request larger than the trajectory yields what exists, not an error.
    assert len(trajectory_to_segments(waypoints, 0.1, 99)) == 7


def test_invalid_dt_is_refused():
    """Check a non-positive or non-finite dt cannot produce a command."""
    waypoints = [(0.0, 0.0, 0.0), (0.05, 0.0, 0.0)]
    for bad in (0.0, -0.1, float('nan'), float('inf')):
        with pytest.raises(ValueError):
            trajectory_to_segments(waypoints, bad, 1)


def test_turns_take_the_short_way_round():
    """Check a yaw step across the wrap point does not spin the long way."""
    segments = trajectory_to_segments(
        [(0.0, 0.0, 3.0), (0.0, 0.0, -3.0)], dt=0.1, max_segments=1
    )
    # The short way is +0.283 rad, not -6.0 rad.
    assert segments[0].angular_z == pytest.approx(
        wrap_angle(-3.0 - 3.0) / 0.1
    )
    assert abs(segments[0].angular_z) < 3.0


def test_clamping_bounds_every_axis():
    """Check no axis can exceed its configured limit."""
    segment = VelocitySegment(
        linear_x=100.0, linear_y=-50.0, angular_z=25.0, duration=0.1
    )
    limited = clamp_segment(segment, 0.2, 0.15, 0.5)
    assert limited.linear_x == pytest.approx(0.2)
    assert limited.linear_y == pytest.approx(-0.15)
    assert limited.angular_z == pytest.approx(0.5)
    assert limited.duration == pytest.approx(0.1)


def test_clamping_maps_non_finite_values_to_zero():
    """Check NaN and infinity become a stop rather than propagating."""
    segment = VelocitySegment(
        linear_x=float('nan'),
        linear_y=float('inf'),
        angular_z=float('-inf'),
        duration=0.1,
    )
    limited = clamp_segment(segment, 0.2, 0.2, 0.5)
    assert limited.linear_x == 0.0
    assert limited.linear_y == 0.0
    assert limited.angular_z == 0.0
