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
    balanced_candidate_order,
    estimate_refined_aruco_pose,
)


def _observation(index=0, margin=100.0):
    return MarkerObservation(
        center_px=(640.0 + 0.1 * index, 360.0 - 0.1 * index),
        margin_px=margin,
        side_px=100.0,
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


def test_marker_window_rejects_small_marker_images():
    values = [_observation(index) for index in range(10)]
    values[-1] = MarkerObservation(
        center_px=values[-1].center_px,
        margin_px=values[-1].margin_px,
        side_px=20.0,
        translation=values[-1].translation,
        rotation_vector=values[-1].rotation_vector,
        received_monotonic=values[-1].received_monotonic)
    valid, message = stable_marker_window(
        values, required_frames=10, min_distance_m=0.25,
        max_distance_m=0.80, minimum_margin_px=60.0,
        minimum_marker_side_px=90.0, maximum_center_std_px=4.0,
        maximum_depth_std_m=0.003, maximum_angle_std_deg=0.8)
    assert not valid and 'small' in message


def test_marker_window_reports_observed_distance_range():
    values = [_observation(index) for index in range(10)]
    values[-1] = MarkerObservation(
        center_px=values[-1].center_px,
        margin_px=values[-1].margin_px,
        side_px=values[-1].side_px,
        translation=(0.0, 0.0, 0.20),
        rotation_vector=values[-1].rotation_vector,
        received_monotonic=values[-1].received_monotonic)
    valid, message = stable_marker_window(
        values, required_frames=10, min_distance_m=0.25,
        max_distance_m=0.80, minimum_margin_px=60.0,
        maximum_center_std_px=4.0, maximum_depth_std_m=0.003,
        maximum_angle_std_deg=0.8)
    assert not valid
    assert 'observed=[0.200, 0.501]m' in message
    assert 'required=[0.250, 0.800]m' in message


def test_candidate_order_interleaves_view_families():
    candidates = [
        SimpleNamespace(candidate_id='roll_-14'),
        SimpleNamespace(candidate_id='roll_-8'),
        SimpleNamespace(candidate_id='horizontal_-10'),
        SimpleNamespace(candidate_id='horizontal_+10'),
    ]
    assert [candidate.candidate_id for candidate in balanced_candidate_order(candidates)] == [
        'roll_-14', 'horizontal_-10', 'roll_-8', 'horizontal_+10']


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
        '    def _verify_simulation_ground_truth(', 1)[0]
    assert run_session.index('self._move_to_initial_anchor(') < \
        run_session.index('self._wait_inputs()')
    assert run_session.index('self._wait_inputs()') < \
        run_session.index('self._wait_easy_services()')
    assert run_session.index('self._wait_easy_services()') < \
        run_session.index('self._clear_easy_samples()')


def test_collector_waits_for_moveit_before_anchor_search():
    source = (
        Path(__file__).parents[1] / 'wvcsc_calibration' /
        'auto_calibration_collector.py').read_text(encoding='utf-8')
    assert "'startup_service_timeout_sec': 20.0" in source
    assert "self._compute_ik_client = self.create_client(" in source
    assert "self._plan_path_client = self.create_client(" in source
    assert "self._execute_trajectory_client = ActionClient(" in source
    run_session = source.split('    def _run_session(self):', 1)[1].split(
        '    def _solve', 1)[0]
    assert run_session.index('self._wait_moveit_services()') < \
        run_session.index('self._move_to_initial_anchor(')


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


def test_collector_waits_for_a_fresh_position_convergence_window_before_pnp_sampling():
    source = (
        Path(__file__).parents[1] / 'wvcsc_calibration' /
        'auto_calibration_collector.py').read_text(encoding='utf-8')
    assert "'joint_stationary_max_position_delta_rad': 0.0005" in source
    assert 'self._joint_position_history = deque(maxlen=200)' in source
    assert 'self._joint_speed_history = deque(maxlen=200)' in source
    stationary = source.split('    def _wait_for_joint_stationary(self):', 1)[1].split(
        '    def _on_camera_info(', 1)[0]
    assert 'waiting for a fresh joint position window' in stationary
    assert 'position_span={position_span:.6f}rad' in stationary
    assert 'reported_max_speed={speed_text}' in stationary
    run_session = source.split('    def _run_session(self):', 1)[1].split(
        '    def _solve', 1)[0]
    assert run_session.index('stationary, reason = self._wait_for_joint_stationary()') < \
        run_session.index('frames, reason = self._stable_observation(')


def test_collector_uses_only_measured_bootstrap_mount_for_camera_centred_views():
    source = (
        Path(__file__).parents[1] / 'wvcsc_calibration' /
        'auto_calibration_collector.py').read_text(encoding='utf-8')
    run_session = source.split('    def _run_session(self):', 1)[1].split(
        '    def _verify_simulation_ground_truth(', 1)[0]
    assert '_BOOTSTRAP_MINIMUM_SAMPLES = 6' in source
    assert '_BOOTSTRAP_TARGET_SAMPLES = 8' in source
    assert 'generate_camera_centered_candidates(' in run_session
    assert 'self._provisional_camera_model()' in run_session
    assert 'tool_camera_prior=bootstrap' in run_session
    assert '[CALIBRATION][BOOTSTRAP]' in run_session
    assert 'frames, reason = self._stable_observation()' in run_session
    assert '_recenter_marker_if_needed' not in run_session
    provisional = source.split('    def _provisional_camera_model(self):', 1)[1].split(
        '    def _run_session(self):', 1)[0]
    assert 'source=measured_samples' in provisional
    assert 'self._compute_consensus_solution()' in provisional
    assert 'ground_truth' not in provisional
    assert 'ground_truth_translation_m' not in run_session


def test_simulation_config_keeps_marker_position_and_final_truth_gate_only():
    config = (Path(__file__).parents[1] / 'config' /
              'auto_handeye_alicia_sim.yaml').read_text(encoding='utf-8')
    assert 'marker_position_base_m: [0.0, 0.25, 0.002]' in config
    assert 'minimum_corner_margin_px' not in config
    assert 'ground_truth_max_translation_error_m: 0.003' in config
    assert 'ground_truth_max_xy_error_m: 0.002' in config
    assert 'ground_truth_max_rotation_error_deg: 1.0' in config


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
    assert 'required_safe = max(minimum_safe, minimum_samples)' in run_session
    assert 'reachability precondition' in run_session
    assert 'candidates, optimizer, required_safe)' not in run_session
    assert run_session.count('candidates, optimizer)') >= 1
    assert 'sample_plan_pool.extend(safe_plans)' in run_session
    assert 'safe_plans = tuple(sample_plan_pool)' in run_session
    assert 'safe-anchor={anchor.candidate_id}' in run_session
    assert 'self._wait_inputs()' in run_session
    assert 'self._screen_candidate_plans(' in run_session


def test_collector_uses_remaining_safe_plans_after_image_quality_rejections():
    source = (
        Path(__file__).parents[1] / 'wvcsc_calibration' /
        'auto_calibration_collector.py').read_text(encoding='utf-8')
    run_session = source.split('    def _run_session(self):', 1)[1].split(
        '    def _solve', 1)[0]
    assert 'for plan in safe_plans:' in run_session
    assert 'for plan in safe_plans[:maximum_samples]:' not in run_session
    assert 'limits recorded samples, not motion attempts' in run_session
    assert 'if sample_count >= maximum_samples:' in run_session


def test_collector_screens_ik_before_incremental_ompl_planning():
    source = (
        Path(__file__).parents[1] / 'wvcsc_calibration' /
        'auto_calibration_collector.py').read_text(encoding='utf-8')
    assert "'candidate_plan_attempts_per_family': 3" in source
    screening = source.split('    def _screen_candidate_plans(', 1)[1].split(
        '    def _execute_candidate_plan(', 1)[0]
    assert 'self._candidate_ik_details(' in screening
    assert 'required_count=None' in screening
    assert 'ranked_families' in screening
    assert 'rank + 1 >= attempts_per_family' in screening
    assert 'len(plans) >= plan_limit' in screening
    assert 'self._checked_candidate_plan(' in screening
    assert "'ik_safe': 0" in screening
    assert "'planned_ok': 0" in screening
    assert "'rejected_condition': 0" in screening


def test_collector_reuses_plans_only_when_start_joints_still_match():
    source = (
        Path(__file__).parents[1] / 'wvcsc_calibration' /
        'auto_calibration_collector.py').read_text(encoding='utf-8')
    execution = source.split('    def _execute_candidate_plan(', 1)[1].split(
        '    def _move_to_initial_anchor(', 1)[0]
    assert 'plan.start_joints' in execution
    assert '<= 0.01' in execution
    assert 'self._safe_plan(plan.candidate, optimizer)' in execution


def test_collector_uses_marker_prior_for_the_first_safe_anchor():
    source = (
        Path(__file__).parents[1] / 'wvcsc_calibration' /
        'auto_calibration_collector.py').read_text(encoding='utf-8')
    assert "'marker_position_base_m': [0.0, 0.25, 0.0]" in source
    anchor = source.split('    def _move_to_initial_anchor(', 1)[1].split(
        '    def _install_calibration_surface(', 1)[0]
    assert 'generate_initial_anchor_candidates(' in anchor
    assert 'self._candidate_ik_details(' in anchor
    assert 'self._checked_candidate_plan(' in anchor
    assert "'no collision-safe initial anchor" in anchor


def test_collector_initial_anchor_scores_condition_before_joint_margin():
    source = (
        Path(__file__).parents[1] / 'wvcsc_calibration' /
        'auto_calibration_collector.py').read_text(encoding='utf-8')
    anchor = source.split('    def _move_to_initial_anchor(', 1)[1].split(
        '    def _install_calibration_surface(', 1)[0]
    assert 'seed_height_candidates_m' in anchor
    assert 'seed_radial_backoff_candidates_m' in anchor
    assert 'seed_tangential_offset_candidates_m' in anchor
    assert 'item.condition' in anchor
    assert '-item.margin' in anchor
    assert anchor.index('item.condition') < anchor.index('-item.margin')
    assert 'joint_margin={selected.margin:.2f}rad' in anchor


def test_simulation_truth_gate_uses_3mm_total_translation_and_2mm_xy_limits():
    source = (
        Path(__file__).parents[1] / 'wvcsc_calibration' /
        'auto_calibration_collector.py').read_text(encoding='utf-8')
    verification = source.split('    def _verify_simulation_ground_truth(', 1)[1].split(
        '    def _solve_and_export(', 1)[0]
    assert 'translation_error <= translation_limit' in verification
    assert 'abs(deltas[0]) <= xy_limit' in verification
    assert 'abs(deltas[1]) <= xy_limit' in verification


def test_aruco_overlay_does_not_overwrite_rclpy_parameter_storage():
    source = (
        Path(__file__).parents[1] / 'wvcsc_calibration' /
        'visualize_aruco_marker.py').read_text(encoding='utf-8')
    assert 'self._detector_parameters = (' in source
    assert 'parameters=self._detector_parameters' in source
    assert 'self._parameters =' not in source


def test_collector_bootstraps_in_tool_space_before_camera_centred_sampling():
    source = (
        Path(__file__).parents[1] / 'wvcsc_calibration' /
        'auto_calibration_collector.py').read_text(encoding='utf-8')
    run_session = source.split('    def _run_session(self):', 1)[1].split(
        '    def _solve', 1)[0]
    assert 'current_tool_position' in run_session
    assert 'current_tool_quaternion' in run_session
    assert 'generate_alicia_candidates(' in run_session
    assert 'tool_camera_prior=false' in run_session
    assert 'generate_camera_centered_candidates(' in run_session
    assert run_session.index('generate_alicia_candidates(') < \
        run_session.index('generate_camera_centered_candidates(')
    anchor = source.split('    def _move_to_initial_anchor(', 1)[1].split(
        '    def _install_calibration_surface(', 1)[0]
    assert 'tool_z_down=true' in anchor
    assert 'tool_camera[0]' not in anchor
    assert 'tool_camera[1]' not in anchor


def test_collector_uses_wvcsc_opencv_transform_conversion_not_server_solver():
    source = (
        Path(__file__).parents[1] / 'wvcsc_calibration' /
        'auto_calibration_collector.py').read_text(encoding='utf-8')
    solver = source.split('    def _compute_consensus_solution(self):', 1)[1].split(
        '    def destroy_node(self):', 1)[0]
    assert 'solve_handeye(' in solver
    assert 'refine_handeye_fixed_marker(' in solver
    assert 'fixed-marker refinement' in solver
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
