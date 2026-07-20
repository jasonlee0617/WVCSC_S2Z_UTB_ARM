from types import SimpleNamespace

import pytest

from wvcsc_calibration.calibration_quality import (
    MarkerObservation,
    calibration_consensus,
    marker_pose_residuals,
    marker_pose_rms,
    pose_is_diverse,
    sample_coverage,
    stable_marker_window,
)


def _observation(index=0, margin=100.0):
    return MarkerObservation(
        center_px=(640.0 + 0.1 * index, 360.0 - 0.1 * index),
        margin_px=margin,
        translation=(0.0, 0.0, 0.50 + 0.0001 * index),
        rotation_vector=(0.0, 0.01, 0.0),
        received_monotonic=float(index))


def test_marker_window_enforces_count_edge_and_stability():
    values = [_observation(index) for index in range(10)]
    valid, _message = stable_marker_window(
        values, required_frames=10, min_distance_m=0.25,
        max_distance_m=0.80, minimum_margin_px=60.0,
        maximum_center_std_px=4.0, maximum_depth_std_m=0.003,
        maximum_angle_std_deg=0.8)
    assert valid
    invalid, message = stable_marker_window(
        values[:-1] + [_observation(9, margin=20.0)],
        required_frames=10, min_distance_m=0.25,
        max_distance_m=0.80, minimum_margin_px=60.0,
        maximum_center_std_px=4.0, maximum_depth_std_m=0.003,
        maximum_angle_std_deg=0.8)
    assert not invalid and 'edge' in message


def test_pose_diversity_and_coverage_use_translation_or_rotation():
    identity = (0.0, 0.0, 0.0, 1.0)
    accepted = [((0.0, 0.0, 0.0), identity)]
    assert not pose_is_diverse(
        (0.001, 0.0, 0.0), identity, accepted, 0.006, 3.0)
    assert pose_is_diverse(
        (0.010, 0.0, 0.0), identity, accepted, 0.006, 3.0)
    translation, rotation = sample_coverage([
        ((0.0, 0.0, 0.0), identity),
        ((0.05, 0.0, 0.0), (0.0, 0.0, 0.173648, 0.984808)),
    ])
    assert translation == pytest.approx(0.05)
    assert rotation == pytest.approx(20.0, abs=1.0e-3)


def test_algorithm_consensus_selects_medoid_and_reports_spread():
    transforms = [
        ((0.10, 0.00, 0.00), (0.0, 0.0, 0.0, 1.0)),
        ((0.102, 0.00, 0.00), (0.0, 0.0, 0.004, 0.999992)),
        ((0.104, 0.00, 0.00), (0.0, 0.0, 0.008, 0.999968)),
    ]
    selected, translation, rotation = calibration_consensus(transforms)
    assert selected == transforms[1]
    assert translation == pytest.approx(0.004)
    assert rotation < 1.0


def _transform(translation=(0.0, 0.0, 0.0), rotation=(0.0, 0.0, 0.0, 1.0)):
    return SimpleNamespace(
        translation=SimpleNamespace(
            x=translation[0], y=translation[1], z=translation[2]),
        rotation=SimpleNamespace(
            x=rotation[0], y=rotation[1], z=rotation[2], w=rotation[3]))


def test_marker_rms_is_zero_for_consistent_composed_samples():
    sample = SimpleNamespace(
        robot=_transform((0.1, 0.0, 0.0)),
        tracking=_transform((0.4, 0.0, 0.0)))
    rms_position, rms_rotation = marker_pose_rms(
        [sample, sample],
        ((0.05, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)))
    assert rms_position == pytest.approx(0.0)
    assert rms_rotation == pytest.approx(0.0)


def test_marker_residuals_expose_one_bad_tracking_sample():
    good = SimpleNamespace(
        robot=_transform((0.1, 0.0, 0.0)),
        tracking=_transform((0.4, 0.0, 0.0)))
    outlier = SimpleNamespace(
        robot=_transform((0.1, 0.0, 0.0)),
        tracking=_transform((0.46, 0.0, 0.0)))
    residuals = marker_pose_residuals(
        [good, good, outlier],
        ((0.05, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)))
    assert residuals[0][0] == pytest.approx(0.0)
    assert residuals[1][0] == pytest.approx(0.0)
    assert residuals[2][0] == pytest.approx(0.06)
