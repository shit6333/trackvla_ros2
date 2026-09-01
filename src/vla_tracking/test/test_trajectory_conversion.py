"""Tests for the geometry that ultimately turns wheels."""

import math

import pytest

from vla_tracking.trajectory_conversion import (
    limit_segment,
    limiting_factor,
    scale_segment,
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


def test_scaling_converts_without_capping():
    """
    Check the scales convert and nothing else.

    A prediction above 1.0 must come out proportionally larger, not cut. The
    checkpoint has no output activation, so it can produce such a value, and
    cutting it here would silently change the direction the model asked for
    while looking like a limit doing its job.
    """
    segment = VelocitySegment(
        linear_x=2.0, linear_y=-0.5, angular_z=1.5, duration=0.1
    )
    scaled = scale_segment(segment, 3.75, 2.5, 1.57)
    assert scaled.linear_x == pytest.approx(7.5)
    assert scaled.linear_y == pytest.approx(-1.25)
    assert scaled.angular_z == pytest.approx(2.355)
    assert scaled.duration == pytest.approx(0.1)


def test_each_axis_carries_its_own_scale():
    """Check the axes are converted independently, as the model normalizes them."""
    segment = VelocitySegment(
        linear_x=1.0, linear_y=1.0, angular_z=1.0, duration=0.1
    )
    scaled = scale_segment(segment, 3.0, 2.0, 1.0)
    assert (scaled.linear_x, scaled.linear_y, scaled.angular_z) == (3.0, 2.0, 1.0)


def test_a_segment_within_the_limits_is_untouched():
    """Check limiting does nothing when there is nothing to limit."""
    segment = VelocitySegment(
        linear_x=0.5, linear_y=0.1, angular_z=0.4, duration=0.1
    )
    assert limiting_factor(segment, 1.0, 1.0, 1.0) == pytest.approx(1.0)
    limited = limit_segment(segment, 1.0, 1.0, 1.0)
    assert limited == segment


def test_limiting_preserves_the_turning_radius():
    """
    Check the whole command is scaled by one factor, not clipped per axis.

    This is the property the redesign exists for. The ratio between linear and
    angular velocity is the turning radius, so clipping one axis and not the
    other puts the robot on an arc the model never asked for. Scaling both
    keeps the arc and only slows the traverse.
    """
    segment = VelocitySegment(
        linear_x=4.0, linear_y=0.0, angular_z=1.0, duration=0.1
    )
    limited = limit_segment(segment, 1.0, 1.0, 2.0)

    assert limited.linear_x == pytest.approx(1.0), 'the saturating axis is at its limit'
    assert limited.angular_z == pytest.approx(0.25), 'the other axis came down with it'
    assert limited.angular_z / limited.linear_x == pytest.approx(
        segment.angular_z / segment.linear_x
    ), 'the turning radius changed'


def test_limiting_uses_the_worst_axis():
    """Check the factor is set by whichever axis is furthest outside."""
    segment = VelocitySegment(
        linear_x=2.0, linear_y=0.0, angular_z=10.0, duration=0.1
    )
    # linear needs 0.5, angular needs 0.2; the smaller must win.
    assert limiting_factor(segment, 1.0, 1.0, 2.0) == pytest.approx(0.2)


def test_a_zero_limit_disables_that_axis_check():
    """Check a limit of zero means unbounded rather than always saturated."""
    segment = VelocitySegment(
        linear_x=9.0, linear_y=0.0, angular_z=0.0, duration=0.1
    )
    assert limiting_factor(segment, 0.0, 0.0, 0.0) == pytest.approx(1.0)


def test_limiting_maps_non_finite_values_to_a_stop():
    """Check NaN and infinity stop the base rather than propagating."""
    for bad in (float('nan'), float('inf'), float('-inf')):
        segment = VelocitySegment(
            linear_x=bad, linear_y=0.1, angular_z=0.1, duration=0.1
        )
        assert limiting_factor(segment, 1.0, 1.0, 1.0) == 0.0
        limited = limit_segment(segment, 1.0, 1.0, 1.0)
        assert limited.linear_x == 0.0
        assert limited.linear_y == 0.0
        assert limited.angular_z == 0.0
