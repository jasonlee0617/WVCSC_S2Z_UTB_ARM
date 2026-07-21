import pytest

from wvcsc_calibration.marker_tf import average_marker_pose


def test_average_marker_pose_averages_translation_and_quaternion_signs():
    translation, quaternion = average_marker_pose((
        ((0.10, 0.20, 0.30), (0.0, 0.0, 0.0, 1.0)),
        ((0.12, 0.18, 0.33), (0.0, 0.0, 0.0, -1.0)),
    ))

    assert translation == pytest.approx((0.11, 0.19, 0.315))
    assert quaternion == pytest.approx((0.0, 0.0, 0.0, 1.0))


def test_average_marker_pose_rejects_an_empty_window():
    with pytest.raises(ValueError, match='at least one'):
        average_marker_pose(())
