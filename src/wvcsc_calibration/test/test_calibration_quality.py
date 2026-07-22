from types import SimpleNamespace
from pathlib import Path

import pytest
import cv2
import numpy as np

from wvcsc_calibration.calibration_quality import (
    MarkerObservation,
    calibration_consensus,
    marker_pose_residuals,
    marker_pose_rms,
    pose_is_diverse,
    sample_coverage,
    stable_marker_window,
    transform_error,
)
from wvcsc_calibration.auto_calibration_collector import (
    camera_center_aim_offsets,
    estimate_refined_aruco_pose,
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


def test_camera_center_aim_offsets_use_live_camera_info():
    matrix = np.asarray((
        (1079.11172, 0.0, 656.42746),
        (0.0, 1082.95708, 525.74486),
        (0.0, 0.0, 1.0),
    ))
    yaw, pitch = camera_center_aim_offsets(
        (matrix, np.zeros(5), 1280, 720))
    assert yaw == pytest.approx(0.862, abs=0.002)
    assert pitch == pytest.approx(-8.702, abs=0.002)
    assert camera_center_aim_offsets(
        (np.asarray(((1000.0, 0.0, 640.0),
                     (0.0, 1000.0, 360.0),
                     (0.0, 0.0, 1.0))), np.zeros(5), 1280, 720)
    ) == pytest.approx((0.0, 0.0))


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


def test_transform_error_is_zero_for_quaternion_sign_equivalence():
    actual = ((-0.055, 0.0, -0.10), (0.0, 0.0, 0.70710678, -0.70710678))
    expected = ((-0.055, 0.0, -0.10), (0.0, 0.0, -0.70710678, 0.70710678))
    translation, rotation = transform_error(actual, expected)
    assert translation == pytest.approx(0.0)
    assert rotation == pytest.approx(0.0)


def test_transform_error_reports_translation_and_rotation_in_public_units():
    translation, rotation = transform_error(
        ((0.003, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
        ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)))
    assert translation == pytest.approx(0.003)
    assert rotation == pytest.approx(0.0)


def test_collector_waits_for_marker_only_after_the_adaptive_anchor_move():
    """The initial operator pose need not see the vehicle-mounted marker."""
    source = (
        Path(__file__).parents[1] / 'wvcsc_calibration' /
        'auto_calibration_collector.py').read_text(encoding='utf-8')
    guarded = source.split('    def _run_session_guarded(self):', 1)[1].split(
        '    def _parameter_string(', 1)[0]
    assert 'self._wait_robot_inputs()' in guarded
    assert 'self._wait_inputs()' not in guarded
    run_session = source.split('    def _run_session(self):', 1)[1].split(
        '    def _solve', 1)[0]
    assert run_session.index('self._move_to_initial_anchor(') < \
        run_session.index('self._wait_inputs()')
    assert run_session.index('self._wait_inputs()') < \
        run_session.index('self._clear_easy_samples()')


def test_collector_does_not_overwrite_rclpy_node_client_storage():
    source = (
        Path(__file__).parents[1] / 'wvcsc_calibration' /
        'auto_calibration_collector.py').read_text(encoding='utf-8')
    assert 'self._easy_clients = self._create_easy_clients()' in source
    assert 'self._clients = self._create_easy_clients()' not in source


def test_collector_uses_best_effort_qos_for_camera_streams():
    source = (
        Path(__file__).parents[1] / 'wvcsc_calibration' /
        'auto_calibration_collector.py').read_text(encoding='utf-8')
    assert 'reliability=ReliabilityPolicy.BEST_EFFORT' in source
    assert 'self._on_camera_info, sensor_qos' in source
    assert 'self._on_image, sensor_qos' in source


def test_collector_shutdown_keeps_motion_cancel_best_effort():
    source = (
        Path(__file__).parents[1] / 'wvcsc_calibration' /
        'auto_calibration_collector.py').read_text(encoding='utf-8')
    method = source.split('    def _request_session_stop(self, reason):', 1)[1].split(
        '    def _run_session_guarded(', 1)[0]
    assert 'try:' in method
    assert 'self._arm.cancel()' in method
    assert 'except Exception:' in method
    assert "reason not in {'Ctrl+C', 'node shutdown'}" in method


def test_collector_recovers_from_a_visible_but_near_singular_seed():
    source = (
        Path(__file__).parents[1] / 'wvcsc_calibration' /
        'auto_calibration_collector.py').read_text(encoding='utf-8')
    run_session = source.split('    def _run_session(self):', 1)[1].split(
        '    def _solve', 1)[0]
    assert 'safe_anchor_recovery_limit' in run_session
    assert "candidate.candidate_id != 'seed'" in run_session
    assert 'include_fine=True' in run_session
    assert 'required_safe = max(minimum_safe, target_samples)' in run_session
    assert 'safe-anchor={anchor.candidate_id}' in run_session
    assert 'self._wait_inputs()' in run_session
    assert 'self._safe_candidates(candidates, optimizer)' in run_session


def test_collector_uses_marker_prior_for_the_first_safe_anchor():
    source = (
        Path(__file__).parents[1] / 'wvcsc_calibration' /
        'auto_calibration_collector.py').read_text(encoding='utf-8')
    assert "'marker_position_base_m': [0.0, 0.25, 0.0]" in source
    anchor = source.split('    def _move_to_initial_anchor(', 1)[1].split(
        '    def _install_calibration_surface(', 1)[0]
    assert 'generate_initial_anchor_candidates(' in anchor
    assert 'self._safe_plan_details(candidate, optimizer)' in anchor
    assert "'no collision-safe initial anchor" in anchor


def test_collector_applies_live_camera_centre_aim_to_all_candidate_sets():
    source = (
        Path(__file__).parents[1] / 'wvcsc_calibration' /
        'auto_calibration_collector.py').read_text(encoding='utf-8')
    run_session = source.split('    def _run_session(self):', 1)[1].split(
        '    def _solve', 1)[0]
    assert 'requested_aim = camera_center_aim_offsets(camera_info)' in run_session
    assert '_anchor_aim = self._move_to_initial_anchor(' in run_session
    assert "'camera_centering_scale_candidates'" in run_session
    assert run_session.count('aim_yaw_deg=aim_offsets[0]') == 2
    anchor = source.split('    def _move_to_initial_anchor(', 1)[1].split(
        '    def _install_calibration_surface(', 1)[0]
    assert 'camera_centering_scale_candidates' in anchor
    assert 'aim_yaw_deg=aim_offsets[0]' in anchor


def test_collector_uses_wvcsc_opencv_transform_conversion_not_server_solver():
    source = (
        Path(__file__).parents[1] / 'wvcsc_calibration' /
        'auto_calibration_collector.py').read_text(encoding='utf-8')
    solver = source.split('    def _compute_consensus_solution(self):', 1)[1].split(
        '    def destroy_node(self):', 1)[0]
    assert 'solve_handeye(' in solver
    assert "'compute'" not in solver
    assert "'set_algorithm'" not in solver


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


def test_refined_square_pnp_recovers_a_synthetic_marker_pose():
    camera = np.asarray(
        ((900.0, 0.0, 640.0), (0.0, 900.0, 360.0), (0.0, 0.0, 1.0)))
    size = 0.070
    half = size * 0.5
    object_points = np.asarray((
        (-half, half, 0.0), (half, half, 0.0),
        (half, -half, 0.0), (-half, -half, 0.0)), dtype=np.float32)
    expected_rvec = np.asarray((0.12, -0.18, 0.06), dtype=float).reshape(3, 1)
    expected_tvec = np.asarray((0.03, -0.02, 0.46), dtype=float).reshape(3, 1)
    corners, _jacobian = cv2.projectPoints(
        object_points, expected_rvec, expected_tvec, camera, np.zeros(5))

    rvec, tvec = estimate_refined_aruco_pose(
        corners.reshape(4, 2), size, camera, np.zeros(5))

    assert tuple(tvec) == pytest.approx(tuple(expected_tvec.reshape(3)), abs=1.0e-6)
    assert tuple(rvec.reshape(3)) == pytest.approx(
        tuple(expected_rvec.reshape(3)), abs=1.0e-6)
