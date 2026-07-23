from pathlib import Path

import pytest

from wvcsc_arm_task.observation import rotate_vector
from wvcsc_calibration.alicia_sample_geometry import tool_orientation_toward_marker


PACKAGE_ROOT = Path(__file__).parents[1]
NODE_SOURCE = (PACKAGE_ROOT / 'wvcsc_calibration' / 'initial_calibration_pose.py').read_text(
    encoding='utf-8')
CALIBRATE_SOURCE = (PACKAGE_ROOT / 'launch' / 'calibrate.launch.py').read_text(
    encoding='utf-8')
EVALUATE_SOURCE = (PACKAGE_ROOT / 'launch' / 'evaluate.launch.py').read_text(
    encoding='utf-8')


def _unit(vector):
    norm = sum(value * value for value in vector) ** 0.5
    return tuple(value / norm for value in vector)


def test_tool_orientation_points_local_z_toward_marker():
    tool = (0.10, -0.20, 0.30)
    marker = (0.50, 0.10, 0.60)
    quaternion = tool_orientation_toward_marker(tool, marker)

    assert rotate_vector((0.0, 0.0, 1.0), quaternion) == pytest.approx(
        _unit(tuple(marker[index] - tool[index] for index in range(3))))


def test_tool_orientation_uses_base_y_when_base_x_is_parallel():
    quaternion = tool_orientation_toward_marker((0.0, 0.0, 0.0), (1.0, 0.0, 0.0))

    assert rotate_vector((0.0, 0.0, 1.0), quaternion) == pytest.approx((1.0, 0.0, 0.0))
    assert rotate_vector((1.0, 0.0, 0.0), quaternion) == pytest.approx((0.0, 1.0, 0.0))


def test_tool_orientation_rejects_coincident_positions():
    with pytest.raises(ValueError, match='invalid'):
        tool_orientation_toward_marker((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))


def test_initial_pose_node_contains_no_vision_or_candidate_search_logic():
    forbidden = (
        'ArucoMarkers', 'CameraInfo', 'Image,', 'TransformListener',
        'rough_tool_to_camera', 'candidate_positions',
        'ObservationOptimizer', 'condition_number', 'joint_margin',
        'stationary', 'take_sample',
    )
    assert all(token not in NODE_SOURCE for token in forbidden)


def test_launches_keep_shared_initial_pose_node_with_requested_defaults():
    for source in (CALIBRATE_SOURCE, EVALUATE_SOURCE):
        assert "executable='initial_calibration_pose'" in source
        assert "'initial_pose_config'" in source
        assert "LaunchConfiguration('initial_pose_config')" in source
    assert "'initial_pose_enabled', default_value='false'" in CALIBRATE_SOURCE
    assert "'initial_pose_enabled', default_value='true'" in EVALUATE_SOURCE
