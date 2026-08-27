"""Tests for the admission checks that stand between a model and the wheels."""

from vla_tracking.trajectory_validation import validate_trajectory

from vla_tracking_interfaces.msg import VlaTrajectory, Waypoint2D

NANOSECONDS_PER_SECOND = 1_000_000_000
NOW_NS = 1_000 * NANOSECONDS_PER_SECOND


def make_trajectory(
    *,
    valid=True,
    dt=0.1,
    frame_id='base_link',
    waypoints=None,
    stamp_ns=NOW_NS,
):
    """Build a trajectory that passes every check unless told otherwise."""
    message = VlaTrajectory()
    message.header.frame_id = frame_id
    message.header.stamp.sec = int(stamp_ns // NANOSECONDS_PER_SECOND)
    message.header.stamp.nanosec = int(stamp_ns % NANOSECONDS_PER_SECOND)
    message.backend_name = 'fake'
    message.dt = dt
    message.valid = valid
    message.waypoints = waypoints if waypoints is not None else [
        Waypoint2D(x=0.0, y=0.0, theta=0.0),
        Waypoint2D(x=0.05, y=0.0, theta=0.0),
    ]
    return message


def test_a_well_formed_trajectory_is_accepted():
    """Check the happy path passes."""
    result = validate_trajectory(make_trajectory(), now_ns=NOW_NS)
    assert result.accepted, result.reason


def test_backend_invalid_is_refused():
    """Check warm-up and failed predictions never execute."""
    result = validate_trajectory(
        make_trajectory(valid=False), now_ns=NOW_NS
    )
    assert not result
    assert 'invalid' in result.reason


def test_backend_invalid_can_be_allowed_explicitly():
    """Check the gate can be opened deliberately for bench work."""
    result = validate_trajectory(
        make_trajectory(valid=False), now_ns=NOW_NS, require_valid=False
    )
    assert result.accepted


def test_too_few_waypoints_is_refused():
    """Check an origin alone cannot be executed."""
    result = validate_trajectory(
        make_trajectory(waypoints=[Waypoint2D()]), now_ns=NOW_NS
    )
    assert not result
    assert 'waypoints' in result.reason


def test_non_positive_dt_is_refused():
    """Check dt cannot divide by zero downstream."""
    for bad in (0.0, -0.1, float('nan')):
        result = validate_trajectory(
            make_trajectory(dt=bad), now_ns=NOW_NS
        )
        assert not result
        assert 'dt' in result.reason


def test_wrong_frame_is_refused():
    """Check a trajectory meant for another frame is not executed."""
    result = validate_trajectory(
        make_trajectory(frame_id='odom'), now_ns=NOW_NS, base_frame='base_link'
    )
    assert not result
    assert 'frame_id' in result.reason


def test_frame_check_can_be_disabled():
    """Check deployments that remap frames elsewhere can opt out."""
    result = validate_trajectory(
        make_trajectory(frame_id='anything'), now_ns=NOW_NS, base_frame=''
    )
    assert result.accepted


def test_non_finite_waypoints_are_refused():
    """Check NaN cannot reach the conversion arithmetic."""
    for bad_field in ('x', 'y', 'theta'):
        waypoint = Waypoint2D(x=0.05, y=0.0, theta=0.0)
        setattr(waypoint, bad_field, float('nan'))
        result = validate_trajectory(
            make_trajectory(
                waypoints=[Waypoint2D(), waypoint]
            ),
            now_ns=NOW_NS,
        )
        assert not result
        assert bad_field in result.reason


def test_stale_observations_are_refused():
    """Check an upstream that republishes an old frame cannot drive the base."""
    old = NOW_NS - int(1.5 * NANOSECONDS_PER_SECOND)
    result = validate_trajectory(
        make_trajectory(stamp_ns=old), now_ns=NOW_NS, max_age_seconds=0.5
    )
    assert not result
    assert 'old' in result.reason


def test_staleness_check_can_be_disabled():
    """Check an unsynchronised camera clock can be tolerated deliberately."""
    old = NOW_NS - int(3600 * NANOSECONDS_PER_SECOND)
    result = validate_trajectory(
        make_trajectory(stamp_ns=old), now_ns=NOW_NS, max_age_seconds=0.0
    )
    assert result.accepted


def test_unset_stamp_is_refused_when_age_matters():
    """Check a missing stamp is not silently treated as fresh."""
    result = validate_trajectory(
        make_trajectory(stamp_ns=0), now_ns=NOW_NS, max_age_seconds=0.5
    )
    assert not result
    assert 'stamp' in result.reason
