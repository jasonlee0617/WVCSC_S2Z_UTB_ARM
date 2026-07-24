from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from wvcsc_calibration.auto_calibration_collector import (
    AutoCalibrationCollector,
    estimate_refined_aruco_pose,
)
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


COLLECTOR_SOURCE = (
    Path(__file__).parents[1] / 'wvcsc_calibration' /
    'auto_calibration_collector.py'
).read_text(encoding='utf-8')


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


def test_fixed_joint_sampler_executes_the_official_order_without_an_anchor_move():
    run_session = COLLECTOR_SOURCE.split('    def _run_session(self):', 1)[1].split(
        '    def _verify_simulation_ground_truth(', 1)[0]
    assert run_session.index('self._wait_moveit_services()') < \
        run_session.index('fixed_samples = fixed_joint_samples()')
    assert 'fixed_samples = fixed_joint_samples()' in run_session
    assert 'for index, joints in enumerate(fixed_samples, start=1):' in run_session
    assert run_session.index('self._fixed_joint_safety(') < \
        run_session.index('self._arm.move_joints(joints)')
    assert run_session.index('self._arm.move_joints(joints)') < \
        run_session.index('self._wait_easy_services()') < \
        run_session.index('self._clear_easy_samples()') < \
        run_session.index("self._call('take', TakeSample.Request())")
    assert 'if sample_count >= minimum_samples:' in run_session
    assert '_wait_inputs()' not in run_session
    assert 'FollowJointTrajectory' not in COLLECTOR_SOURCE


def test_fixed_joint_safety_rejects_condition_and_margin_before_moveit():
    class FakeCollector:
        def get_parameter(self, name):
            values = {
                'joint_names': ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'],
                'max_condition_number': 14.0,
                'min_joint_margin_rad': 0.22,
            }
            return SimpleNamespace(value=values[name])

    class FakeOptimizer:
        def __init__(self, condition, margin):
            self._condition = condition
            self._margin = margin

        def condition_number(self, _joints):
            return self._condition

        def minimum_joint_margin(self, _joints):
            return self._margin

    collector = FakeCollector()
    valid = AutoCalibrationCollector._fixed_joint_safety(
        collector, (0.0,) * 6, FakeOptimizer(10.0, 0.30))
    assert valid == (True, 'accepted', 10.0, 0.30)
    assert AutoCalibrationCollector._fixed_joint_safety(
        collector, (0.0,) * 6, FakeOptimizer(14.0, 0.30))[1] == 'near_singularity'
    assert AutoCalibrationCollector._fixed_joint_safety(
        collector, (0.0,) * 6, FakeOptimizer(10.0, 0.21))[1] == 'joint_limit_margin'


def test_adaptive_candidate_and_bootstrap_paths_are_absent():
    forbidden = (
        'generate_alicia_candidates', 'generate_camera_centered_candidates',
        'generate_initial_anchor_candidates', '_move_to_initial_anchor',
        '_provisional_camera_model', '_screen_candidate_plans',
        'safe_anchor_recovery_limit', 'target_samples', 'maximum_samples',
        'minimum_safe_candidates', 'candidate_plan_attempts_per_family',
        'seed_height_candidates_m', 'seed_radial_backoff_candidates_m',
        'seed_tangential_offset_candidates_m', '_BOOTSTRAP_',
    )
    assert all(name not in COLLECTOR_SOURCE for name in forbidden)


def test_collector_keeps_c10_qos_cancel_and_stationary_safety_contracts():
    assert 'reliability=ReliabilityPolicy.BEST_EFFORT' in COLLECTOR_SOURCE
    assert 'self._on_camera_info, sensor_qos' in COLLECTOR_SOURCE
    assert 'self._on_image, sensor_qos' in COLLECTOR_SOURCE
    assert 'self._easy_clients = self._create_easy_clients()' in COLLECTOR_SOURCE
    assert 'self._clients = self._create_easy_clients()' not in COLLECTOR_SOURCE
    assert 'self._joint_position_history = deque(maxlen=200)' in COLLECTOR_SOURCE
    assert 'waiting for a fresh joint position window' in COLLECTOR_SOURCE
    assert 'self._arm.cancel()' in COLLECTOR_SOURCE


def test_simulation_config_keeps_coverage_marker_and_fixed_minimum():
    config = (Path(__file__).parents[1] / 'config' /
              'auto_handeye_alicia_sim.yaml').read_text(encoding='utf-8')
    assert 'marker_position_base_m: [0.530, -0.030, 0.002]' in config
    assert 'minimum_samples: 14' in config
    assert 'minimum_solution_samples: 14' in config
    assert 'ground_truth_max_translation_error_m: 0.003' in config
    assert 'ground_truth_max_xy_error_m: 0.002' in config
    assert 'ground_truth_max_rotation_error_deg: 1.0' in config
    real_config = (Path(__file__).parents[1] / 'config' /
                   'auto_handeye_alicia.yaml').read_text(encoding='utf-8')
    assert 'max_condition_number: 35.0' in real_config
    assert 'seed_height_candidates_m' not in config
    assert 'target_samples' not in config


def test_simulation_truth_gate_uses_3mm_total_translation_and_2mm_xy_limits():
    verification = COLLECTOR_SOURCE.split(
        '    def _verify_simulation_ground_truth(', 1)[1].split(
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


def test_collector_uses_wvcsc_opencv_transform_conversion_not_server_solver():
    solver = COLLECTOR_SOURCE.split('    def _compute_consensus_solution(self):', 1)[1].split(
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
