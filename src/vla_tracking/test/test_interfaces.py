"""
Contract tests for the generated tracking interfaces.

Every node, adapter, and recorded bag depends on these field names, types,
and constants, so a silent rename must fail here rather than at runtime.
"""

import array

from rclpy.serialization import deserialize_message, serialize_message

from vla_tracking_interfaces.action import TrackTarget
from vla_tracking_interfaces.msg import VlaStatus, VlaTrajectory, Waypoint2D


def test_waypoint_fields():
    """Check that a waypoint carries the three planar values the model emits."""
    assert Waypoint2D.get_fields_and_field_types() == {
        'x': 'double',
        'y': 'double',
        'theta': 'double',
    }


def test_trajectory_fields():
    """Check that a trajectory exposes frame, timing, origin and validity."""
    fields = VlaTrajectory.get_fields_and_field_types()
    assert fields['header'] == 'std_msgs/Header'
    assert fields['backend_name'] == 'string'
    assert fields['waypoints'] == \
        'sequence<vla_tracking_interfaces/Waypoint2D>'
    assert fields['dt'] == 'float'
    assert fields['valid'] == 'boolean'
    assert fields['status'] == 'string'


def test_trajectory_defaults_are_not_executable():
    """
    Check that an unpopulated trajectory never looks executable.

    Publishing a default-constructed message is a bug, and the executor's
    validity check depends on `valid` defaulting to False.
    """
    msg = VlaTrajectory()
    assert msg.valid is False
    assert list(msg.waypoints) == []
    assert msg.dt == 0.0


def test_trajectory_round_trip():
    """Check that waypoint values survive serialization unchanged."""
    msg = VlaTrajectory()
    msg.header.frame_id = 'base_link'
    msg.backend_name = 'fake'
    msg.dt = 0.1
    msg.valid = True
    msg.waypoints = [
        Waypoint2D(x=0.0, y=0.0, theta=0.0),
        Waypoint2D(x=0.0486, y=0.0015, theta=-0.0034),
    ]

    restored = deserialize_message(serialize_message(msg), VlaTrajectory)

    assert restored.header.frame_id == 'base_link'
    assert restored.backend_name == 'fake'
    assert restored.valid is True
    assert len(restored.waypoints) == 2
    assert restored.waypoints[1].x == 0.0486
    assert restored.waypoints[1].theta == -0.0034
    # dt is a float32 field, so compare it at single precision.
    assert array.array('f', [0.1])[0] == restored.dt


def test_status_task_states_are_distinct():
    """Check the five documented task states exist and do not collide."""
    states = {
        'IDLE': VlaStatus.IDLE,
        'WARMING_UP': VlaStatus.WARMING_UP,
        'TRACKING': VlaStatus.TRACKING,
        'ERROR': VlaStatus.ERROR,
        'STOPPED': VlaStatus.STOPPED,
    }
    assert len(set(states.values())) == len(states)
    assert VlaStatus().task_state == VlaStatus.IDLE


def test_status_fields():
    """Check that status reports task state, readiness and timing."""
    fields = VlaStatus.get_fields_and_field_types()
    assert fields['task_state'] == 'uint8'
    assert fields['instruction'] == 'string'
    assert fields['backend_name'] == 'string'
    assert fields['backend_ready'] == 'boolean'
    assert fields['last_inference_duration'] == 'float'
    assert fields['last_observation_stamp'] == 'builtin_interfaces/Time'
    assert fields['predictions_published'] == 'uint32'
    assert fields['error_message'] == 'string'


def test_action_shape():
    """Check the action takes an instruction and reports status both ways."""
    assert TrackTarget.Goal.get_fields_and_field_types() == {
        'instruction': 'string',
    }

    result_fields = TrackTarget.Result.get_fields_and_field_types()
    assert result_fields['final_status'] == 'vla_tracking_interfaces/VlaStatus'
    assert result_fields['predictions_published'] == 'uint32'
    assert result_fields['message'] == 'string'

    feedback_fields = TrackTarget.Feedback.get_fields_and_field_types()
    assert feedback_fields['status'] == 'vla_tracking_interfaces/VlaStatus'
