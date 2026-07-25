import math

import pytest

from wvcsc_calibration.alicia_sample_geometry import (
    ALICIA_M_FIXED_JOINT_SAMPLES,
    fixed_joint_samples,
)


_OFFICIAL_ALICIA_SEQUENCE = (
    (0.0, -1.09, -0.87, 0.0, -0.77, 0.0),
    (-0.12, -1.09, -0.87, 0.0, -0.77, 0.0),
    (0.12, -1.09, -0.87, 0.0, -0.77, 0.0),
    (-0.20, -1.09, -0.87, 0.0, -0.77, 0.0),
    (0.0, -1.09, -0.87, 0.0, -0.77, 0.4),
    (0.0, -1.09, -0.87, 0.0, -0.77, -0.4),
    (0.0, -1.09, -0.87, 0.0, -0.77, 0.8),
    (0.0, -1.09, -0.87, 0.0, -0.65, 0.0),
    (0.0, -1.09, -0.87, 0.0, -0.90, 0.0),
    (0.0, -1.09, -0.87, 0.0, -0.55, 0.0),
    (0.0, -1.09, -0.87, 0.25, -0.77, 0.0),
    (0.0, -1.09, -0.87, -0.25, -0.77, 0.0),
    (0.0, -1.09, -0.87, 0.4, -0.77, 0.0),
    (0.15, -1.09, -0.87, 0.0, -0.77, 0.5),
    (-0.15, -1.09, -0.87, 0.0, -0.77, -0.5),
    (0.10, -1.09, -0.87, 0.0, -0.77, -0.6),
    (0.0, -1.09, -0.87, 0.2, -0.70, 0.3),
    (0.0, -1.09, -0.87, -0.2, -0.85, -0.3),
    (0.0, -1.09, -0.87, 0.15, -0.60, 0.5),
    (-0.10, -1.09, -0.87, -0.15, -0.70, -0.4),
)


def test_fixed_joint_sequence_matches_every_official_pose_and_order():
    """Protect the explicit one-to-one migration from Alicia-M calibration."""
    assert ALICIA_M_FIXED_JOINT_SAMPLES == _OFFICIAL_ALICIA_SEQUENCE
    assert fixed_joint_samples() == _OFFICIAL_ALICIA_SEQUENCE
    assert len(ALICIA_M_FIXED_JOINT_SAMPLES) == 20
    assert all(len(pose) == 6 for pose in ALICIA_M_FIXED_JOINT_SAMPLES)
    assert all(math.isfinite(value)
               for pose in ALICIA_M_FIXED_JOINT_SAMPLES for value in pose)


def test_fixed_joint_sequence_is_immutable():
    with pytest.raises(TypeError):
        ALICIA_M_FIXED_JOINT_SAMPLES[0][0] = 1.0
