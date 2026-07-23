#!/usr/bin/env python3
"""Interactive, Alicia-M-adaptive eye-in-hand calibration collector.

The launch terminal owns hardware, MoveIt, C10, ArUco and easy_handeye2.  This
second-terminal process owns only candidate generation and sample collection.
Press ``s`` or Enter to start one fresh session and ``q`` to cancel it.  Any
external stop/reset event invalidates the whole session; after HOME and resume,
the operator must press ``s`` again.
"""

from collections import deque
from dataclasses import dataclass
import math
import select
import sys
import threading
import time

import cv2
from cv_bridge import CvBridge
from easy_handeye2_msgs.srv import (
    RemoveSample,
    SaveSamples,
    TakeSample,
)
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.srv import GetMotionPlan, GetPositionIK
import numpy as np
import rclpy
from rcl_interfaces.srv import GetParameters
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener

from wvcsc_arm_task.motion_state import MotionControlState
from wvcsc_arm_task.node_parameters import create_alicia_moveit
from wvcsc_arm_task.observation import (
    ObservationOptimizer,
)

from .alicia_sample_geometry import (
    generate_alicia_candidates,
    generate_camera_centered_candidates,
    generate_initial_anchor_candidates,
)
from .calibration_io import write_calibration, write_calibration_outputs
from .calibration_quality import (
    MarkerObservation,
    calibration_consensus,
    compose_transforms,
    marker_pose_rms,
    marker_pose_residuals,
    pose_is_diverse,
    sample_coverage,
    stable_marker_window,
    transform_error,
    transform_components,
)
from .calibration_solver import refine_handeye_fixed_marker, solve_handeye
from .calibration_solver import matrix_quaternion
from .marker_tf import average_marker_pose


_BOOTSTRAP_MINIMUM_SAMPLES = 6
_BOOTSTRAP_TARGET_SAMPLES = 8


def _wait_future(future, timeout_sec):
    deadline = time.monotonic() + float(timeout_sec)
    while rclpy.ok() and not future.done() and time.monotonic() < deadline:
        time.sleep(0.01)
    return future.done()


def _transform_tuple(stamped):
    transform = stamped.transform
    return transform_components(transform)


def estimate_refined_aruco_pose(corners, marker_size_m, camera_matrix, distortion):
    """Estimate a square-marker pose with the planar IPPE solver when present.

    ArUco's convenience API is suitable for continuous visualization, but a
    hand-eye sample benefits from the square-specific IPPE solution after its
    corners have been refined to sub-pixel precision.  The marker coordinate
    convention matches ``estimatePoseSingleMarkers``: origin at the printed
    square centre, +X right and +Y up when viewing the code face.  A robust
    fallback preserves compatibility with OpenCV builds that lack IPPE.
    """
    marker_size = float(marker_size_m)
    if not math.isfinite(marker_size) or marker_size <= 0.0:
        raise ValueError('marker_size_m must be finite and positive')
    image_points = np.asarray(corners, dtype=np.float32).reshape(4, 2)
    half = marker_size * 0.5
    object_points = np.asarray((
        (-half, half, 0.0), (half, half, 0.0),
        (half, -half, 0.0), (-half, -half, 0.0),
    ), dtype=np.float32)
    try:
        result = cv2.solvePnPGeneric(
            object_points, image_points, camera_matrix, distortion,
            flags=cv2.SOLVEPNP_IPPE_SQUARE)
        solved, rotations, translations, reprojection = result[:4]
        if solved and rotations and translations:
            errors = (np.asarray(reprojection, dtype=float).reshape(-1)
                      if reprojection is not None else
                      np.full(len(rotations), math.inf))
            candidates = []
            for index, (rotation, translation) in enumerate(
                    zip(rotations, translations)):
                vector = np.asarray(translation, dtype=float).reshape(3)
                if vector[2] > 0.0 and np.all(np.isfinite(vector)):
                    candidates.append((
                        float(errors[index]) if index < len(errors) else math.inf,
                        np.asarray(rotation, dtype=float).reshape(3, 1), vector))
            if candidates:
                _error, rotation, translation = min(candidates, key=lambda item: item[0])
                return rotation, translation
    except cv2.error:
        pass
    rotations, translations, _objects = cv2.aruco.estimatePoseSingleMarkers(
        np.asarray([image_points], dtype=np.float32), marker_size,
        camera_matrix, distortion)
    return (np.asarray(rotations[0], dtype=float).reshape(3, 1),
            np.asarray(translations[0], dtype=float).reshape(3))


def _candidate_family(candidate_id):
    name = str(candidate_id)
    for family in (
            'seed', 'roll', 'wide_roll', 'wide_orbit', 'horizontal',
            'vertical', 'wide_tilt', 'combo', 'radial', 'fine'):
        if name == family or name.startswith(f'{family}_'):
            return family
    return 'other'


def _candidate_identifier(candidate):
    if hasattr(candidate, 'candidate'):
        return getattr(candidate.candidate, 'candidate_id')
    return getattr(candidate, 'candidate_id')


def balanced_candidate_order(candidates):
    """Interleave view families so accepted samples do not cluster by tilt."""
    buckets = {}
    order = []
    for candidate in candidates:
        family = _candidate_family(_candidate_identifier(candidate))
        if family not in buckets:
            buckets[family] = []
            order.append(family)
        buckets[family].append(candidate)
    ordered = []
    while any(buckets.values()):
        for family in order:
            if buckets[family]:
                ordered.append(buckets[family].pop(0))
    return ordered


@dataclass(frozen=True)
class CandidatePlan:
    candidate: object
    trajectory: object
    start_joints: tuple
    condition: float
    margin: float
    joint_motion: float


class AutoCalibrationCollector(Node):
    """Generate, safety-filter, execute and solve one Alicia-M sample set."""

    def __init__(self):
        super().__init__('auto_calibration_collector')
        self._declare_parameters()
        self._group = ReentrantCallbackGroup()
        self._state = MotionControlState()
        self._arm, self._arm_group = create_alicia_moveit(self, self._state)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._bridge = CvBridge()
        self._data_lock = threading.Lock()
        self._session_lock = threading.Lock()
        self._session_thread = None
        self._session_cancel = threading.Event()
        self._session_invalid = threading.Event()
        self._auto_start_pending = bool(self.get_parameter('auto_start').value)
        self._external_locked = False
        self._joint_positions = None
        self._joint_position_history = deque(maxlen=200)
        self._joint_speed_history = deque(maxlen=200)
        self._camera_info = None
        self._observations = deque(maxlen=max(
            2 * int(self.get_parameter('stable_frames').value), 20))

        self._dictionary = self._create_dictionary()
        self._detector_parameters = (
            cv2.aruco.DetectorParameters()
            if hasattr(cv2.aruco, 'DetectorParameters')
            else cv2.aruco.DetectorParameters_create())
        # ``rclpy.node.Node`` reserves ``_clients`` for its own ROS client
        # entities.  Keep easy_handeye2 clients in a distinctly named mapping;
        # overwriting Node._clients prevents executors from spinning at all.
        self._easy_clients = self._create_easy_clients()
        self._compute_ik_client = self.create_client(
            GetPositionIK, '/compute_ik', callback_group=self._group)
        self._plan_path_client = self.create_client(
            GetMotionPlan, '/plan_kinematic_path',
            callback_group=self._group)
        self._execute_trajectory_client = ActionClient(
            self, ExecuteTrajectory, '/execute_trajectory',
            callback_group=self._group)
        self._stable_marker_publisher = self.create_publisher(
            PoseStamped,
            str(self.get_parameter('stable_marker_pose_topic').value), 1)
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        # Gazebo C10 and real camera drivers commonly publish image streams as
        # BEST_EFFORT.  A default RELIABLE subscription is incompatible and
        # silently starves the collector of CameraInfo/image frames.
        sensor_qos = QoSProfile(
            depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            JointState, str(self.get_parameter('joint_state_topic').value),
            self._on_joint_state, 10, callback_group=self._group)
        self.create_subscription(
            CameraInfo, str(self.get_parameter('camera_info_topic').value),
            self._on_camera_info, sensor_qos, callback_group=self._group)
        self.create_subscription(
            Image, str(self.get_parameter('image_topic').value),
            self._on_image, sensor_qos, callback_group=self._group)
        self.create_subscription(
            Bool, str(self.get_parameter('motion_locked_topic').value),
            self._on_motion_locked, latched, callback_group=self._group)
        self.create_subscription(
            String, str(self.get_parameter('motion_state_topic').value),
            self._on_motion_state, latched, callback_group=self._group)

        # 日常标定仍坚持由独立终端的 s/Enter 启动，避免机械臂在操作者
        # 未确认工作区安全时自行运动。``auto_start`` 只用于无 TTY 的
        # Gazebo 回归验证；它保留同一套采集与安全状态机，而非另建测试路径。
        if self._auto_start_pending:
            self._auto_start_timer = self.create_timer(
                0.1, self._start_automatic_session, callback_group=self._group)
        else:
            self._keyboard_thread = threading.Thread(
                target=self._keyboard_loop, daemon=True)
            self._keyboard_thread.start()
        self.get_logger().info(
            '[CALIBRATION] Alicia-M collector ready: s/Enter=start, q=cancel. '
            'Use motion_control_keyboard for SPACE/h/r arm control.')

    def _declare_parameters(self):
        values = {
            'base_frame': 'alicia_base_link',
            'tool_link': 'tool0',
            'camera_frame': 'camera_color_optical_frame',
            'marker_frame': 'calibration_aruco',
            'joint_names': [f'joint{index}' for index in range(1, 7)],
            'marker_id': 1,
            'marker_size_m': 0.070,
            'aruco_dictionary': 'DICT_5X5_250',
            'image_topic': '/camera/color/image_raw',
            'camera_info_topic': '/camera/color/camera_info',
            'joint_state_topic': '/joint_states',
            'motion_locked_topic': '/motion_control/locked',
            'motion_state_topic': '/motion_control/state',
            'minimum_samples': 15,
            'minimum_solution_samples': 14,
            'target_samples': 18,
            'maximum_samples': 22,
            'minimum_safe_candidates': 14,
            'safe_anchor_recovery_limit': 2,
            'stable_frames': 10,
            'settle_time_sec': 1.0,
            # A planar marker can look stable while the wrist is still
            # coasting.  Require a fresh joint-position convergence window
            # before associating image PnP with the latest robot TF sample.
            'joint_stationary_max_position_delta_rad': 0.0005,
            'joint_stationary_window_sec': 0.30,
            'joint_stationary_timeout_sec': 5.0,
            'marker_distance_min_m': 0.25,
            'marker_distance_max_m': 0.80,
            'minimum_corner_margin_px': 60.0,
            'minimum_marker_side_px': 90.0,
            'maximum_center_std_px': 4.0,
            'maximum_marker_depth_std_m': 0.003,
            'maximum_marker_angle_std_deg': 0.8,
            'minimum_translation_delta_m': 0.006,
            'minimum_rotation_delta_deg': 3.0,
            'minimum_translation_span_m': 0.04,
            'minimum_rotation_span_deg': 20.0,
            'max_condition_number': 14.0,
            'min_joint_margin_rad': 0.22,
            'position_tolerance_m': 0.003,
            'orientation_tolerance_rad': 0.01,
            'algorithm_names': [
                'OpenCV/Park', 'OpenCV/Horaud', 'OpenCV/Tsai-Lenz'],
            'maximum_algorithm_translation_delta_m': 0.010,
            'maximum_algorithm_rotation_delta_deg': 2.0,
            'maximum_camera_translation_norm_m': 0.30,
            'maximum_marker_position_rms_m': 0.005,
            'maximum_marker_rotation_rms_deg': 1.0,
            # Refine the deterministic OpenCV seed by requiring every sample
            # to imply one fixed base->marker pose.  This is a measurement
            # consistency objective, not a simulation-truth correction.
            'fixed_marker_refinement_enabled': True,
            'fixed_marker_refinement_translation_sigma_m': 0.001,
            'fixed_marker_refinement_rotation_sigma_deg': 1.0,
            'fixed_marker_refinement_max_iterations': 25,
            'easy_service_timeout_sec': 10.0,
            'startup_service_timeout_sec': 20.0,
            'candidate_plan_attempts_per_family': 3,
            'output_file': (
                '$HOME/WVCSC_S2Z_UTB_ARM/src/wvcsc_calibration/config/'
                'c10_handeye.yaml'),
            'easy_handeye_output_file': (
                '~/.ros2/easy_handeye2/calibrations/wvcsc_c10.calib'),
            'marker_position_base_m': [0.0, 0.25, 0.0],
            'seed_height_candidates_m': [0.30, 0.35, 0.40, 0.45, 0.50],
            'seed_radial_backoff_candidates_m': [0.05, 0.10, 0.15, 0.20],
            'seed_tangential_offset_candidates_m': [
                0.0, -0.05, 0.05, -0.10, 0.10],
            'calibration_surface_enabled': False,
            'calibration_surface_id': 'calibration_surface',
            'calibration_surface_frame': 'alicia_base_link',
            'calibration_surface_size_m': [0.50, 0.80, 0.04],
            'calibration_surface_position_m': [0.35, 0.0, -0.03],
            'ground_truth_check_enabled': False,
            'ground_truth_translation_m': [-0.055, 0.0, -0.10],
            'ground_truth_rotation_xyzw': [
                0.0, 0.0, -0.7071067811865476, 0.7071067811865476],
            'ground_truth_max_translation_error_m': 0.004,
            'ground_truth_max_xy_error_m': 0.002,
            'ground_truth_max_rotation_error_deg': 1.0,
            # The collector owns the exact pose used for each sample.  Its
            # quality-gated stable window is forwarded to the only marker-TF
            # broadcaster immediately before easy_handeye2 records a sample.
            'stable_marker_pose_topic': '/calibration/stable_marker_pose',
            'stable_tf_settle_sec': 0.15,
            # 仅供无交互终端的仿真回归使用；实机配置必须保持 false，
            # 继续要求操作者按 s/Enter 进行明确确认。
            'auto_start': False,
        }
        for name, default in values.items():
            if not self.has_parameter(name):
                self.declare_parameter(name, default)
        minimum = int(self.get_parameter('minimum_samples').value)
        solution_minimum = int(
            self.get_parameter('minimum_solution_samples').value)
        target = int(self.get_parameter('target_samples').value)
        maximum = int(self.get_parameter('maximum_samples').value)
        if not 3 <= solution_minimum <= minimum <= target <= maximum:
            raise ValueError(
                'sample limits must satisfy 3 <= solution_min <= min <= '
                'target <= max')
        if int(self.get_parameter('safe_anchor_recovery_limit').value) < 0:
            raise ValueError('safe_anchor_recovery_limit must be non-negative')
        stable_tf_settle_sec = float(
            self.get_parameter('stable_tf_settle_sec').value)
        if (not math.isfinite(stable_tf_settle_sec)
                or stable_tf_settle_sec < 0.0):
            raise ValueError('stable_tf_settle_sec must be finite and non-negative')
        startup_timeout = float(
            self.get_parameter('startup_service_timeout_sec').value)
        if not math.isfinite(startup_timeout) or startup_timeout <= 0.0:
            raise ValueError('startup_service_timeout_sec must be finite and positive')
        if int(self.get_parameter('candidate_plan_attempts_per_family').value) <= 0:
            raise ValueError('candidate_plan_attempts_per_family must be positive')
        stationary_values = tuple(float(self.get_parameter(name).value) for name in (
            'joint_stationary_max_position_delta_rad',
            'joint_stationary_window_sec',
            'joint_stationary_timeout_sec'))
        if not all(math.isfinite(value) and value > 0.0
                   for value in stationary_values):
            raise ValueError('joint stationary parameters must be finite and positive')
        quality_limits = (
            float(self.get_parameter(
                'maximum_algorithm_translation_delta_m').value),
            float(self.get_parameter(
                'maximum_algorithm_rotation_delta_deg').value),
            float(self.get_parameter(
                'maximum_marker_position_rms_m').value),
            float(self.get_parameter(
                'maximum_marker_rotation_rms_deg').value),
        )
        if (not all(math.isfinite(value) for value in quality_limits)
                or min(quality_limits) <= 0.0):
            raise ValueError('calibration quality limits must be finite and positive')
        marker_position = tuple(float(value) for value in self.get_parameter(
            'marker_position_base_m').value)
        if (len(marker_position) != 3
                or not all(math.isfinite(value) for value in marker_position)
                or math.hypot(marker_position[0], marker_position[1]) < 0.05):
            raise ValueError(
                'marker_position_base_m must contain a finite X/Y offset from base')
        for name, positive in (
                ('seed_height_candidates_m', True),
                ('seed_radial_backoff_candidates_m', False),
                ('seed_tangential_offset_candidates_m', False)):
            values = tuple(float(value) for value in self.get_parameter(name).value)
            if (not values or not all(math.isfinite(value) for value in values)
                    or (positive and min(values) <= 0.0)
                    or (not positive and name == 'seed_radial_backoff_candidates_m'
                        and min(values) < 0.0)):
                raise ValueError(f'{name} contains invalid anchor candidates')
        for name, expected_size in (
                ('ground_truth_translation_m', 3),
                ('ground_truth_rotation_xyzw', 4)):
            values = tuple(float(value) for value in self.get_parameter(name).value)
            if (len(values) != expected_size
                    or not all(math.isfinite(value) for value in values)):
                raise ValueError(
                    f'{name} must contain {expected_size} finite values')

    def _create_dictionary(self):
        name = str(self.get_parameter('aruco_dictionary').value)
        if not hasattr(cv2, 'aruco') or not hasattr(cv2.aruco, name):
            raise RuntimeError(f'OpenCV ArUco dictionary is unavailable: {name}')
        return cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))

    def _create_easy_clients(self):
        services = {
            'take': (TakeSample, '/easy_handeye2/calibration/take_sample'),
            'get': (TakeSample, '/easy_handeye2/calibration/get_sample_list'),
            'remove': (
                RemoveSample, '/easy_handeye2/calibration/remove_sample'),
            'save_samples': (
                SaveSamples, '/easy_handeye2/calibration/save_samples'),
        }
        return {
            name: self.create_client(interface, service, callback_group=self._group)
            for name, (interface, service) in services.items()
        }

    def _wait_moveit_services(self):
        timeout = float(self.get_parameter('startup_service_timeout_sec').value)
        self.get_logger().info(
            '[CALIBRATION] waiting for MoveIt services')
        for name, client in (
                ('/compute_ik', self._compute_ik_client),
                ('/plan_kinematic_path', self._plan_path_client)):
            if not client.wait_for_service(timeout_sec=timeout):
                raise RuntimeError(f'MoveIt service unavailable: {name}')
        if not self._execute_trajectory_client.wait_for_server(
                timeout_sec=timeout):
            raise RuntimeError(
                'MoveIt action server unavailable: /execute_trajectory')

    def _wait_easy_services(self):
        timeout = float(self.get_parameter('easy_service_timeout_sec').value)
        self.get_logger().info(
            '[CALIBRATION] waiting for easy_handeye2 services')
        for name, client in self._easy_clients.items():
            if not client.wait_for_service(timeout_sec=timeout):
                raise RuntimeError(f'easy_handeye2 service unavailable: {name}')

    def _on_joint_state(self, message):
        values = dict(zip(message.name, message.position))
        names = tuple(str(value) for value in self.get_parameter('joint_names').value)
        try:
            positions = tuple(float(values[name]) for name in names)
        except (KeyError, TypeError, ValueError):
            return
        if all(math.isfinite(value) for value in positions):
            with self._data_lock:
                self._joint_positions = positions
                self._joint_position_history.append((time.monotonic(), positions))
                if len(message.velocity) == len(message.name):
                    speeds = dict(zip(message.name, message.velocity))
                    try:
                        maximum_speed = max(
                            abs(float(speeds[name])) for name in names)
                    except (KeyError, TypeError, ValueError):
                        return
                    if math.isfinite(maximum_speed):
                        self._joint_speed_history.append(
                            (time.monotonic(), maximum_speed))

    def _clear_joint_stationary_history(self):
        with self._data_lock:
            self._joint_position_history.clear()
            self._joint_speed_history.clear()

    def _wait_for_joint_stationary(self):
        """Require a fresh converged joint-position window before PnP."""
        maximum_position_delta = float(self.get_parameter(
            'joint_stationary_max_position_delta_rad').value)
        required_window = float(self.get_parameter(
            'joint_stationary_window_sec').value)
        deadline = time.monotonic() + float(self.get_parameter(
            'joint_stationary_timeout_sec').value)
        last_reason = 'waiting for joint position samples'
        while rclpy.ok() and time.monotonic() < deadline:
            if self._session_cancel.is_set():
                return False, 'session canceled'
            now = time.monotonic()
            with self._data_lock:
                position_history = tuple(self._joint_position_history)
                speed_history = tuple(self._joint_speed_history)
            window = tuple(item for item in position_history
                           if item[0] >= now - required_window)
            if (len(window) >= 2
                    and window[-1][0] - window[0][0] >= required_window * 0.90):
                position_span = max(
                    max(item[1][joint] for item in window) -
                    min(item[1][joint] for item in window)
                    for joint in range(len(window[0][1])))
                speed_window = tuple(item for item in speed_history
                                     if item[0] >= window[0][0])
                reported_speed = (max(item[1] for item in speed_window)
                                  if speed_window else None)
                if position_span <= maximum_position_delta:
                    speed_text = ('unavailable' if reported_speed is None else
                                  f'{reported_speed:.5f}rad/s')
                    self.get_logger().info(
                        '[CALIBRATION] joint stationary: '
                        f'position_span={position_span:.6f}rad '
                        f'window={window[-1][0] - window[0][0]:.2f}s '
                        f'reported_max_speed={speed_text}')
                    return True, 'joint position is stationary'
                last_reason = (
                    f'joint position span {position_span:.6f}rad exceeds '
                    f'{maximum_position_delta:.6f}rad')
            elif position_history:
                last_reason = 'waiting for a fresh joint position window'
            time.sleep(0.02)
        return False, last_reason

    def _on_camera_info(self, message):
        if message.width <= 0 or message.height <= 0:
            return
        camera = np.asarray(message.k, dtype=float).reshape(3, 3)
        if camera[0, 0] <= 0.0 or camera[1, 1] <= 0.0:
            return
        distortion = np.asarray(
            message.d if message.d else [0.0] * 5, dtype=float)
        with self._data_lock:
            self._camera_info = (
                camera, distortion, int(message.width), int(message.height))

    def _on_image(self, message):
        with self._data_lock:
            camera_info = self._camera_info
        if camera_info is None:
            return
        try:
            image = self._bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
            corners, ids, _rejected = cv2.aruco.detectMarkers(
                image, self._dictionary, parameters=self._detector_parameters)
            if ids is None:
                with self._data_lock:
                    self._observations.clear()
                return
            flat = [int(value) for value in np.asarray(ids).reshape(-1)]
            index = flat.index(int(self.get_parameter('marker_id').value))
            marker_corners = np.asarray(corners[index], dtype=np.float32).reshape(4, 2)
            camera, distortion, width, height = camera_info
            # Refine the four corners before PnP.  This is especially
            # important in Gazebo where a 70 mm tabletop code can otherwise
            # move by a whole render pixel between frames; the same operation
            # also benefits the real RGB-only C10 pipeline.
            grayscale = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            refined = marker_corners.reshape(-1, 1, 2).copy()
            cv2.cornerSubPix(
                grayscale, refined, (5, 5), (-1, -1),
                (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
                 30, 0.01))
            marker_corners = refined.reshape(4, 2)
            rotation_vector, translation = estimate_refined_aruco_pose(
                marker_corners,
                float(self.get_parameter('marker_size_m').value),
                camera, distortion)
            centre = np.mean(marker_corners, axis=0)
            margin = min(
                float(np.min(marker_corners[:, 0])),
                float(np.min(marker_corners[:, 1])),
                float(width - np.max(marker_corners[:, 0])),
                float(height - np.max(marker_corners[:, 1])))
            side_px = float(np.mean([
                np.linalg.norm(
                    marker_corners[(index + 1) % 4] - marker_corners[index])
                for index in range(4)
            ]))
            observation = MarkerObservation(
                center_px=(float(centre[0]), float(centre[1])),
                margin_px=margin,
                side_px=side_px,
                translation=tuple(float(value) for value in translation),
                rotation_vector=tuple(float(value) for value in rotation_vector.reshape(3)),
                received_monotonic=time.monotonic())
            with self._data_lock:
                self._observations.append(observation)
        except (ValueError, cv2.error):
            with self._data_lock:
                self._observations.clear()

    def _invalidate_session(self, reason):
        self._session_invalid.set()
        self._session_cancel.set()
        self._state.stop()
        self._arm.cancel()
        self.get_logger().warn(
            f'[CALIBRATION] session invalidated by motion state: {reason}')

    def _on_motion_locked(self, message):
        self._external_locked = bool(message.data)
        if self._external_locked:
            self._invalidate_session('motion locked')
        else:
            self._state.resume()

    def _on_motion_state(self, message):
        state = str(message.data).strip().upper()
        if state in {
                'STOPPED', 'STOPPED_LOCKED', 'RESETTING', 'RESET_FAILED',
                'HOME_LOCKED'}:
            self._invalidate_session(state)
        elif state in {'NORMAL', 'RUNNING', 'READY'}:
            self._state.resume()

    def _keyboard_loop(self):
        if not sys.stdin.isatty():
            self.get_logger().error(
                '[CALIBRATION] stdin is not a TTY; start this node in its own terminal')
            return
        while rclpy.ok():
            readable, _writeable, _errors = select.select([sys.stdin], [], [], 0.2)
            if not readable:
                continue
            command = sys.stdin.readline()
            if command == '':
                return
            command = command.strip().lower()
            if command in {'', 's'}:
                self._start_session()
            elif command == 'q':
                self._request_session_stop('operator q')

    def _start_automatic_session(self):
        """在无 TTY 的 Gazebo 验证中只启动一次相同的采集会话。"""
        if not self._auto_start_pending:
            return
        self._auto_start_pending = False
        self.destroy_timer(self._auto_start_timer)
        self.get_logger().info(
            '[CALIBRATION] auto_start=true; starting simulation session')
        self._start_session()

    def _start_session(self):
        with self._session_lock:
            if self._session_thread is not None and self._session_thread.is_alive():
                self.get_logger().warn('[CALIBRATION] a collection session is active')
                return
            if self._external_locked or self._state.locked:
                self.get_logger().error(
                    '[CALIBRATION] motion is locked; complete h -> HOME_LOCKED -> r first')
                return
            self._session_cancel.clear()
            self._session_invalid.clear()
            self._session_thread = threading.Thread(
                target=self._run_session_guarded, daemon=True)
            self._session_thread.start()

    def _request_session_stop(self, reason):
        # During SIGINT rclpy may have already invalidated rosout.  The caller
        # already owns shutdown reporting, so avoid a misleading second
        # traceback from logging through a destroyed ROS context.
        if reason not in {'Ctrl+C', 'node shutdown'}:
            self.get_logger().warn(f'[CALIBRATION] cancel requested: {reason}')
        self._session_cancel.set()
        # SIGINT may already have invalidated the rclpy context before the
        # collector's finally block runs.  Preserve best-effort cancellation
        # during normal operation, but never turn a clean shutdown into a
        # traceback merely because its publishers have been destroyed.
        try:
            self._arm.cancel()
        except Exception:
            pass

    def _run_session_guarded(self):
        seed = None
        try:
            # 开始姿态通常不会看见安装在车体上的标定码。因此这里只等待
            # 机械臂、相机内参与自身TF，先保存可安全回退的真实起始关节；
            # 标定码TF必须在移动到自适应初始观察位之后才作为会话前置条件。
            seed, _camera, _transforms = self._wait_robot_inputs()
            self._run_session()
        except Exception as error:
            self.get_logger().error(f'[CALIBRATION] session failed: {error}')
        finally:
            if (seed is not None and not self._session_invalid.is_set()
                    and not self._state.locked):
                self.get_logger().info('[CALIBRATION] returning to session start joints')
                if not self._arm.move_joints(seed):
                    self.get_logger().error(
                        '[CALIBRATION] failed to return to session start joints')
            self.get_logger().info(
                '[CALIBRATION] session ended; press s to start a new session')

    def _parameter_string(self, node_name, parameter_name):
        service_name = (
            f'{str(node_name).rstrip("/")}/get_parameters')
        client = self.create_client(
            GetParameters, service_name, callback_group=self._group)
        timeout = float(self.get_parameter('easy_service_timeout_sec').value)
        try:
            if not client.wait_for_service(timeout_sec=timeout):
                raise RuntimeError(
                    f'parameter service unavailable: {node_name}')
            future = client.call_async(
                GetParameters.Request(names=[parameter_name]))
            if not _wait_future(future, timeout):
                raise RuntimeError(
                    f'parameter read timed out: {node_name}/{parameter_name}')
            response = future.result()
            if response is None or not response.values:
                raise RuntimeError(
                    f'parameter is unavailable: {parameter_name}')
            value = str(response.values[0].string_value)
            if not value.strip():
                raise RuntimeError(f'parameter is empty: {parameter_name}')
            return value
        finally:
            self.destroy_client(client)

    def _lookup(self, target, source):
        return self._tf_buffer.lookup_transform(target, source, Time())

    def _wait_inputs(self, timeout_sec=10.0):
        """Wait for robot state and the measured camera-to-marker TF.

        The camera-to-tool transform is deliberately not part of this wait.
        It is the unknown being calibrated and must not influence sampling.
        """
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            if self._session_cancel.is_set():
                raise RuntimeError('session canceled')
            with self._data_lock:
                joints = self._joint_positions
                camera = self._camera_info
            try:
                transforms = (
                    self._lookup(
                        str(self.get_parameter('base_frame').value),
                        str(self.get_parameter('tool_link').value)),
                    self._lookup(
                        str(self.get_parameter('camera_frame').value),
                        str(self.get_parameter('marker_frame').value)),
                )
            except TransformException:
                transforms = None
            if joints is not None and camera is not None and transforms is not None:
                return joints, camera, transforms
            time.sleep(0.05)
        raise RuntimeError('joint state, CameraInfo or calibration TF is unavailable')

    def _wait_robot_inputs(self, timeout_sec=10.0):
        """Wait for inputs needed before moving to the initial tool target.

        初始 HOME 或上一次姿态可能根本不在车顶 marker 视野内，因此不能在
        会话刚开始时等待 ``camera -> marker`` TF。该阶段只确认关节、
        CameraInfo 和当前 ``base -> tool0`` 状态。
        """
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            if self._session_cancel.is_set():
                raise RuntimeError('session canceled')
            with self._data_lock:
                joints = self._joint_positions
                camera = self._camera_info
            try:
                transforms = (
                    self._lookup(
                        str(self.get_parameter('base_frame').value),
                        str(self.get_parameter('tool_link').value)),
                )
            except TransformException:
                transforms = None
            if joints is not None and camera is not None and transforms is not None:
                return joints, camera, transforms
            time.sleep(0.05)
        raise RuntimeError('joint state, CameraInfo or robot/tool TF is unavailable')

    def _optimizer(self):
        description = self._parameter_string(
            '/robot_state_publisher', 'robot_description')
        return ObservationOptimizer(
            description,
            str(self.get_parameter('base_frame').value),
            str(self.get_parameter('tool_link').value),
            tuple(str(value) for value in self.get_parameter('joint_names').value),
            {
                'max_condition_number': float(
                    self.get_parameter('max_condition_number').value),
                'min_joint_margin_rad': float(
                    self.get_parameter('min_joint_margin_rad').value),
                'preferred_joint_margin_rad': float(
                    self.get_parameter('min_joint_margin_rad').value),
            })

    def _current_joints(self):
        with self._data_lock:
            return self._joint_positions

    def _candidate_ik_details(self, candidate, optimizer, joints=None, names=None):
        if joints is None:
            joints = self._current_joints()
        if joints is None:
            return None, 'no_joint_state'
        if names is None:
            names = tuple(str(value) for value in self.get_parameter(
                'joint_names').value)
        solution = self._arm.compute_ik(
            candidate.tool_position, candidate.tool_quaternion, joints, timeout=0.5)
        if solution is None:
            return None, 'ik'
        by_name = dict(zip(solution.name, solution.position))
        try:
            ik_joints = tuple(float(by_name[name]) for name in names)
        except KeyError:
            return None, 'ik'
        ik_condition = optimizer.condition_number(ik_joints)
        ik_margin = optimizer.minimum_joint_margin(ik_joints)
        if ik_condition >= float(
                self.get_parameter('max_condition_number').value):
            return None, 'condition'
        if ik_margin < float(self.get_parameter('min_joint_margin_rad').value):
            return None, 'margin'
        joint_motion = math.sqrt(sum(
            (ik_joints[index] - joints[index]) ** 2
            for index in range(len(joints))))
        return (ik_joints, ik_condition, ik_margin, joint_motion), None

    def _checked_candidate_plan(
            self, candidate, optimizer, joints, names,
            ik_condition, ik_margin):
        trajectory = self._arm.plan_pose(
            candidate.tool_position, candidate.tool_quaternion,
            frame_id=str(self.get_parameter('base_frame').value),
            tolerance_position=float(
                self.get_parameter('position_tolerance_m').value),
            tolerance_orientation=float(
                self.get_parameter('orientation_tolerance_rad').value))
        final = self._arm.trajectory_final_positions(trajectory, names)
        if final is None:
            return None
        final_condition = optimizer.condition_number(final)
        final_margin = optimizer.minimum_joint_margin(final)
        if (final_condition >= float(
                self.get_parameter('max_condition_number').value)
                or final_margin < float(
                    self.get_parameter('min_joint_margin_rad').value)):
            return None
        joint_motion = math.sqrt(sum(
            (final[index] - joints[index]) ** 2 for index in range(len(joints))))
        return CandidatePlan(
            candidate,
            trajectory,
            tuple(float(value) for value in joints),
            max(ik_condition, final_condition),
            min(ik_margin, final_margin),
            joint_motion,
        )

    def _safe_plan_details(self, candidate, optimizer):
        joints = self._current_joints()
        if joints is None:
            return None
        names = tuple(str(value) for value in self.get_parameter(
            'joint_names').value)
        details, _reason = self._candidate_ik_details(
            candidate, optimizer, joints, names)
        if details is None:
            return None
        _ik_joints, ik_condition, ik_margin, _ik_motion = details
        plan = self._checked_candidate_plan(
            candidate, optimizer, joints, names, ik_condition, ik_margin)
        if plan is None:
            return None
        return (
            plan.trajectory,
            plan.condition,
            plan.margin,
            plan.joint_motion,
        )

    def _safe_plan(self, candidate, optimizer):
        details = self._safe_plan_details(candidate, optimizer)
        return details[0] if details is not None else None

    def _candidate_screen_key(self, item):
        condition, margin, joint_motion, candidate = item[:4]
        return (condition, -margin, joint_motion, str(candidate.candidate_id))

    def _screen_candidate_plans(self, candidates, optimizer, required_count=None):
        """IK-screen candidates and expand OMPL plans only when needed."""
        names = tuple(str(value) for value in self.get_parameter(
            'joint_names').value)
        joints = self._current_joints()
        stats = {
            'total': len(candidates),
            'ik_safe': 0,
            'rejected_ik': 0,
            'rejected_condition': 0,
            'rejected_margin': 0,
            'planned_ok': 0,
            'planned_failed': 0,
        }
        if joints is None:
            stats['rejected_ik'] = len(candidates)
            return [], stats
        buckets = {}
        family_order = []
        for candidate in candidates:
            details, reason = self._candidate_ik_details(
                candidate, optimizer, joints, names)
            if details is None:
                if reason == 'condition':
                    stats['rejected_condition'] += 1
                elif reason == 'margin':
                    stats['rejected_margin'] += 1
                else:
                    stats['rejected_ik'] += 1
                continue
            _ik_joints, condition, margin, joint_motion = details
            stats['ik_safe'] += 1
            family = _candidate_family(candidate.candidate_id)
            if family not in buckets:
                buckets[family] = []
                family_order.append(family)
            buckets[family].append(
                (condition, margin, joint_motion, candidate))

        attempts_per_family = int(self.get_parameter(
            'candidate_plan_attempts_per_family').value)
        ranked_families = {
            family: sorted(buckets[family], key=self._candidate_screen_key)
            for family in family_order
        }
        plan_limit = (None if required_count is None
                      else max(1, int(required_count)))
        plans = []
        maximum_rank = max((len(values) for values in ranked_families.values()),
                           default=0)
        for rank in range(maximum_rank):
            for family in family_order:
                ranked = ranked_families[family]
                if rank >= len(ranked):
                    continue
                condition, margin, _motion, candidate = ranked[rank]
                plan = self._checked_candidate_plan(
                    candidate, optimizer, joints, names, condition, margin)
                if plan is None:
                    stats['planned_failed'] += 1
                    continue
                stats['planned_ok'] += 1
                plans.append(plan)
            if rank + 1 >= attempts_per_family and (
                    plan_limit is None or len(plans) >= plan_limit):
                break
        return balanced_candidate_order(plans), stats

    def _execute_candidate_plan(self, plan, optimizer):
        self._clear_joint_stationary_history()
        joints = self._current_joints()
        if (joints is not None and len(joints) == len(plan.start_joints)
                and max(abs(joints[index] - plan.start_joints[index])
                        for index in range(len(joints))) <= 0.01):
            return self._arm.execute_trajectory(plan.trajectory)
        trajectory = self._safe_plan(plan.candidate, optimizer)
        return trajectory is not None and self._arm.execute_trajectory(trajectory)

    def _move_to_initial_anchor(self, optimizer):
        """Move to a safe tool-space marker view before waiting for marker TF."""
        marker_position = tuple(float(value) for value in self.get_parameter(
            'marker_position_base_m').value)
        safe = []
        names = tuple(str(value) for value in self.get_parameter(
            'joint_names').value)
        anchor_plan_attempts = max(
            12,
            4 * int(self.get_parameter(
                'candidate_plan_attempts_per_family').value))
        candidates = generate_initial_anchor_candidates(
            marker_position,
            self.get_parameter('seed_height_candidates_m').value,
            self.get_parameter('seed_radial_backoff_candidates_m').value,
            self.get_parameter('seed_tangential_offset_candidates_m').value)
        joints = self._current_joints()
        if joints is None:
            raise RuntimeError('joint state is unavailable for initial anchor')
        ranked = []
        stats = {
            'total': len(candidates),
            'ik_safe': 0,
            'rejected_ik': 0,
            'rejected_condition': 0,
            'rejected_margin': 0,
            'planned_ok': 0,
            'planned_failed': 0,
        }
        for candidate in candidates:
            details, reason = self._candidate_ik_details(
                candidate, optimizer, joints, names)
            if details is None:
                if reason == 'condition':
                    stats['rejected_condition'] += 1
                elif reason == 'margin':
                    stats['rejected_margin'] += 1
                else:
                    stats['rejected_ik'] += 1
                continue
            _ik_joints, condition, margin, joint_motion = details
            stats['ik_safe'] += 1
            ranked.append((condition, margin, joint_motion, candidate))
        for condition, margin, _motion, candidate in sorted(
                ranked, key=self._candidate_screen_key)[:anchor_plan_attempts]:
            plan = self._checked_candidate_plan(
                candidate, optimizer, joints, names, condition, margin)
            if plan is None:
                stats['planned_failed'] += 1
                continue
            stats['planned_ok'] += 1
            safe.append(plan)
        self.get_logger().info(
            '[CALIBRATION] initial-anchor probe: '
            f'ik_safe={stats["ik_safe"]} '
            f'rejected_ik={stats["rejected_ik"]} '
            f'rejected_condition={stats["rejected_condition"]} '
            f'rejected_margin={stats["rejected_margin"]} '
            f'planned_ok={stats["planned_ok"]} '
            f'planned_failed={stats["planned_failed"]} '
            f'total={stats["total"]} tool_z_down=true')
        if not safe:
            raise RuntimeError(
                'no collision-safe initial anchor for marker_position_base_m')
        # The primary target remains the direct tool-Z-down pose.  If its
        # nominal camera projection is clipped by the unknown camera
        # translation, try the other already-safe tool-space anchors and let
        # the strict image gate choose the first visible one.  This uses no
        # camera extrinsic or Gazebo truth; the only feedback is the measured
        # ArUco image window.
        ordered_safe = sorted(
            safe,
            key=lambda item: (
                0 if '_r0.000_t+0.000' in item.candidate.candidate_id else 1,
                item.condition,
                -item.margin,
                item.joint_motion,
                str(item.candidate.candidate_id)))
        last_reason = 'marker was not visible at any safe initial anchor'
        for selected in ordered_safe:
            candidate = selected.candidate
            self._clear_joint_stationary_history()
            if not self._execute_candidate_plan(selected, optimizer):
                self.get_logger().warn(
                    '[CALIBRATION] initial-anchor execution rejected: '
                    f'{candidate.candidate_id}')
                continue
            self.get_logger().info(
                '[CALIBRATION] initial-anchor='
                f'{candidate.candidate_id} condition={selected.condition:.2f} '
                f'joint_margin={selected.margin:.2f}rad '
                f'joint_motion={selected.joint_motion:.2f}rad '
                'tool_space_prior=true tool_camera_prior=false')
            time.sleep(float(self.get_parameter('settle_time_sec').value))
            stationary, reason = self._wait_for_joint_stationary()
            if not stationary:
                last_reason = reason
                self.get_logger().warn(
                    '[CALIBRATION] initial-anchor did not stop: '
                    f'{candidate.candidate_id}: {reason}')
                continue
            frames, reason = self._stable_observation(timeout_sec=3.0)
            if frames is not None:
                self.get_logger().info(
                    '[CALIBRATION] initial-anchor-visible=true '
                    f'candidate={candidate.candidate_id}')
                return candidate
            last_reason = reason
            self.get_logger().warn(
                '[CALIBRATION] initial-anchor image rejected: '
                f'{candidate.candidate_id}: {reason}')
        raise RuntimeError(
            'no safe initial anchor with a stable visible marker: '
            f'{last_reason}')

    def _install_calibration_surface(self):
        """Publish the target-side tabletop as a collision object for OMPL."""
        if not bool(self.get_parameter('calibration_surface_enabled').value):
            return
        size = tuple(float(value) for value in self.get_parameter(
            'calibration_surface_size_m').value)
        position = tuple(float(value) for value in self.get_parameter(
            'calibration_surface_position_m').value)
        if (len(size) != 3 or len(position) != 3
                or not all(math.isfinite(value) and value > 0.0 for value in size)
                or not all(math.isfinite(value) for value in position)):
            raise RuntimeError('calibration surface parameters are invalid')
        self._arm.add_collision_box(
            str(self.get_parameter('calibration_surface_id').value), size,
            position, str(self.get_parameter('calibration_surface_frame').value))
        # CollisionObject is delivered through a topic; wait briefly before the
        # first collision-aware IK and OMPL request.
        time.sleep(0.25)
        self.get_logger().info(
            '[CALIBRATION] installed target-side tabletop collision geometry')

    def _stable_observation(self, timeout_sec=5.0, minimum_corner_margin_px=None):
        with self._data_lock:
            self._observations.clear()
        deadline = time.monotonic() + timeout_sec
        required = int(self.get_parameter('stable_frames').value)
        if minimum_corner_margin_px is None:
            minimum_corner_margin_px = float(
                self.get_parameter('minimum_corner_margin_px').value)
        last_reason = 'waiting for marker'
        while rclpy.ok() and time.monotonic() < deadline:
            if self._session_cancel.is_set():
                return None, 'session canceled'
            with self._data_lock:
                frames = list(self._observations)
            valid, last_reason = stable_marker_window(
                frames,
                required_frames=required,
                min_distance_m=float(
                    self.get_parameter('marker_distance_min_m').value),
                max_distance_m=float(
                    self.get_parameter('marker_distance_max_m').value),
                minimum_margin_px=float(minimum_corner_margin_px),
                minimum_marker_side_px=float(
                    self.get_parameter('minimum_marker_side_px').value),
                maximum_center_std_px=float(
                    self.get_parameter('maximum_center_std_px').value),
                maximum_depth_std_m=float(
                    self.get_parameter('maximum_marker_depth_std_m').value),
                maximum_angle_std_deg=float(
                    self.get_parameter('maximum_marker_angle_std_deg').value))
            if valid:
                return frames[-required:], last_reason
            time.sleep(0.05)
        return None, last_reason

    def _publish_stable_marker_pose(self, frames):
        """Publish the quality-gated PnP mean used by the next hand-eye sample.

        The returned pose is ``camera_color_optical_frame -> marker``.  The
        separate marker-TF node remains the sole TF authority and temporarily
        forwards this message, so easy_handeye2 sees exactly this window
        rather than a newer raw detector frame from a moving render stream.
        """
        if not frames:
            raise RuntimeError('cannot publish an empty marker sample window')
        poses = []
        for frame in frames:
            matrix, _jacobian = cv2.Rodrigues(
                np.asarray(frame.rotation_vector, dtype=float))
            poses.append((frame.translation, matrix_quaternion(matrix)))
        translation, quaternion = average_marker_pose(poses)
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = str(self.get_parameter('camera_frame').value)
        message.pose.position.x = translation[0]
        message.pose.position.y = translation[1]
        message.pose.position.z = translation[2]
        message.pose.orientation.x = quaternion[0]
        message.pose.orientation.y = quaternion[1]
        message.pose.orientation.z = quaternion[2]
        message.pose.orientation.w = quaternion[3]
        self._stable_marker_publisher.publish(message)
        time.sleep(float(self.get_parameter('stable_tf_settle_sec').value))
        return translation, quaternion

    def _call(self, name, request):
        client = self._easy_clients[name]
        timeout = float(self.get_parameter('easy_service_timeout_sec').value)
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f'easy_handeye2 service unavailable: {name}')
        future = client.call_async(request)
        if not _wait_future(future, timeout):
            raise RuntimeError(f'easy_handeye2 service timed out: {name}')
        response = future.result()
        if response is None:
            raise RuntimeError(f'easy_handeye2 returned no response: {name}')
        return response

    def _clear_easy_samples(self):
        """清空服务端上一轮样本，保证计数、求解和质量门控只属于本轮。

        easy_handeye2 的采样器会在服务端进程生命周期内保留样本。如果直接
        开始下一轮，``take_sample`` 返回的数量会包含旧样本，可能让不足 15 个
        新姿态的会话错误通过门控。这里始终从末尾删除并再次读取确认；任一
        服务调用失败都中止本轮，不使用混合样本继续求解。
        """
        response = self._call('get', TakeSample.Request())
        count = len(response.samples.samples)
        if count:
            self.get_logger().info(
                f'[CALIBRATION] clearing {count} samples from previous session')
        while count:
            response = self._call(
                'remove', RemoveSample.Request(sample_index=count - 1))
            new_count = len(response.samples.samples)
            if new_count != count - 1:
                raise RuntimeError(
                    'easy_handeye2 sample reset made no deterministic progress')
            count = new_count
        verified = self._call('get', TakeSample.Request())
        if verified.samples.samples:
            raise RuntimeError('easy_handeye2 sample reset verification failed')

    def _safe_candidates(self, candidates, optimizer):
        """Return only poses that pass the unchanged collision/IK safety gate."""
        plans, _stats = self._screen_candidate_plans(candidates, optimizer)
        return [plan.candidate for plan in plans]

    def _provisional_camera_model(self):
        """Estimate a planning-only camera mount from bootstrap measurements."""
        (algorithm, handeye, samples, translation_delta,
         rotation_delta) = self._compute_consensus_solution()
        marker_poses = [
            compose_transforms(
                compose_transforms(transform_components(sample.robot), handeye),
                transform_components(sample.tracking))
            for sample in samples]
        marker_pose = average_marker_pose(marker_poses)
        marker_position_rms, marker_rotation_rms = marker_pose_rms(
            samples, handeye)
        self.get_logger().info(
            '[CALIBRATION][PROVISIONAL] '
            f'source=measured_samples algorithm={algorithm} '
            f'samples={len(samples)} algorithm_spread='
            f'{translation_delta * 1000.0:.2f}mm/{rotation_delta:.2f}deg '
            f'marker_rms={marker_position_rms * 1000.0:.2f}mm/'
            f'{marker_rotation_rms:.2f}deg')
        return handeye, marker_pose

    def _run_session(self):
        self._wait_moveit_services()
        self._install_calibration_surface()
        # easy_handeye2 intentionally exposes its service API only after the
        # camera->marker TF exists.  Use the configured base-frame marker
        # prior to reach a collision-safe view first; only then require the
        # camera->marker TF and clear previous samples.
        _joints, _camera_info, _robot_transforms = self._wait_robot_inputs()
        optimizer = self._optimizer()
        self._move_to_initial_anchor(optimizer)
        _joints, _camera, transforms = self._wait_inputs()
        self._wait_easy_services()
        self._clear_easy_samples()
        _base_tool, _camera_marker = transforms
        marker_position = tuple(float(value) for value in self.get_parameter(
            'marker_position_base_m').value)
        current_tool_position, current_tool_quaternion = _transform_tuple(
            _base_tool)
        minimum_safe = int(self.get_parameter('minimum_safe_candidates').value)
        minimum_samples = int(self.get_parameter('minimum_samples').value)
        target_samples = int(self.get_parameter('target_samples').value)
        bootstrap_target = min(_BOOTSTRAP_TARGET_SAMPLES, target_samples)
        # ``target_samples`` improves solution quality but is not a
        # reachability precondition.  Requiring every target view before any
        # motion made valid 18-sample sessions fail with 18--21 safe plans.
        # The sampling loop below still aims for the target and only solves
        # after the configured minimum has been reached.
        required_safe = max(minimum_safe, minimum_samples)
        recovery_limit = int(
            self.get_parameter('safe_anchor_recovery_limit').value)
        candidates = ()
        safe_plans = ()
        sample_plan_pool = []
        sampled_anchor_count = 0
        used_anchor_ids = set()
        for recovery_index in range(recovery_limit + 1):
            candidates = generate_alicia_candidates(
                marker_position, current_tool_position,
                current_tool_quaternion, include_fine=True)
            safe_plans, stats = self._screen_candidate_plans(
                candidates, optimizer)
            self.get_logger().info(
                '[CALIBRATION] tool-space candidate probe: '
                f'ik_safe={stats["ik_safe"]} '
                f'rejected_ik={stats["rejected_ik"]} '
                f'rejected_condition={stats["rejected_condition"]} '
                f'rejected_margin={stats["rejected_margin"]} '
                f'planned_ok={stats["planned_ok"]} '
                f'planned_failed={stats["planned_failed"]} '
                f'total={stats["total"]}')
            sample_plan_pool.extend(safe_plans)
            sampled_anchor_count += 1
            if len(sample_plan_pool) >= required_safe:
                safe_plans = tuple(sample_plan_pool)
                break
            if not safe_plans or recovery_index >= recovery_limit:
                raise RuntimeError(
                    f'only {len(sample_plan_pool)} safe Alicia-M candidates; need '
                    f'{required_safe}')

            # The official Alicia reference pose intentionally favors marker
            # visibility and can be near a wrist singularity.  Move once to a
            # proven-safe marker-facing view, then regenerate the same
            # marker-relative pattern around that non-singular anchor instead
            # of weakening the condition-number or joint-margin limits.
            # Do not select the regenerated ``seed`` candidate once already
            # anchored: it leaves the arm in exactly the same configuration
            # and cannot improve safe-candidate coverage.  Prefer a distinct
            # non-seed perturbation while preserving the existing safety gate.
            eligible = [
                plan for plan in safe_plans
                if plan.candidate.candidate_id != 'seed'
                and plan.candidate.candidate_id not in used_anchor_ids]
            if not eligible:
                eligible = [
                    plan for plan in safe_plans
                    if plan.candidate.candidate_id != 'seed']
            anchor = None
            for plan in eligible:
                candidate = plan.candidate
                used_anchor_ids.add(candidate.candidate_id)
                if not self._execute_candidate_plan(plan, optimizer):
                    self.get_logger().warn(
                        '[CALIBRATION] recovery anchor execution rejected: '
                        f'{candidate.candidate_id}')
                    continue
                anchor = candidate
                break
            if anchor is None:
                raise RuntimeError(
                    'no previously safe calibration anchor remained executable')
            self.get_logger().info(
                f'[CALIBRATION] safe-anchor={anchor.candidate_id} '
                f'attempt={recovery_index + 1}/{recovery_limit}')
            time.sleep(float(self.get_parameter('settle_time_sec').value))
            stationary, reason = self._wait_for_joint_stationary()
            if not stationary:
                raise RuntimeError(
                    f'safe calibration anchor did not stop: {reason}')
            _joints, _camera, transforms = self._wait_inputs()
            _base_tool, _camera_marker = transforms
            current_tool_position, current_tool_quaternion = _transform_tuple(
                _base_tool)
        self.get_logger().info(
            f'[CALIBRATION] {len(safe_plans)} safe candidate plans from '
            f'{sampled_anchor_count} anchor(s) passed collision IK, Jacobian, '
            'joint-margin and OMPL checks; '
            'tool_camera_prior=false')
        safe_plans = balanced_candidate_order(safe_plans)

        accepted_poses = []
        sample_count = 0
        maximum_samples = int(self.get_parameter('maximum_samples').value)
        # Bootstrap samples are collected from tool-space targets only.  They
        # establish a measured provisional mount; no Gazebo mount value is
        # involved in this phase.
        # A collision-safe trajectory can still be rejected later because the
        # marker is too near the image edge or duplicates an already accepted
        # pose.  The strict image gate is intentionally the only camera-based
        # acceptance step; no exact camera mount is used to recenter a target.
        # Keep trying the remaining pre-screened plans until the recorded
        # sample limit (or the target) is actually reached.
        for plan in safe_plans:
            if sample_count >= bootstrap_target:
                break
            candidate = plan.candidate
            if self._session_cancel.is_set() or self._session_invalid.is_set():
                raise RuntimeError('session canceled or invalidated')
            if not self._execute_candidate_plan(plan, optimizer):
                self.get_logger().warn(
                    f'[CALIBRATION] rejected during execution: {candidate.candidate_id}')
                continue
            time.sleep(float(self.get_parameter('settle_time_sec').value))
            stationary, reason = self._wait_for_joint_stationary()
            if not stationary:
                self.get_logger().warn(
                    f'[CALIBRATION] rejected {candidate.candidate_id}: '
                    f'joints did not stop: {reason}')
                continue
            frames, reason = self._stable_observation()
            if frames is None:
                self.get_logger().warn(
                    f'[CALIBRATION] image quality rejected '
                    f'{candidate.candidate_id}: {reason}')
                continue
            current_tool = self._lookup(
                str(self.get_parameter('base_frame').value),
                str(self.get_parameter('tool_link').value))
            pose = _transform_tuple(current_tool)
            if not pose_is_diverse(
                    pose[0], pose[1], accepted_poses,
                    float(self.get_parameter('minimum_translation_delta_m').value),
                    float(self.get_parameter('minimum_rotation_delta_deg').value)):
                self.get_logger().warn(
                    f'[CALIBRATION] diversity rejected {candidate.candidate_id}')
                continue
            self._publish_stable_marker_pose(frames)
            response = self._call('take', TakeSample.Request())
            new_count = len(response.samples.samples)
            if new_count <= sample_count:
                self.get_logger().warn(
                    f'[CALIBRATION] easy_handeye2 rejected {candidate.candidate_id}')
                continue
            sample_count = new_count
            accepted_poses.append(pose)
            self.get_logger().info(
                f'[CALIBRATION][BOOTSTRAP] sample={sample_count}/{bootstrap_target} '
                f'candidate={candidate.candidate_id}')
            if sample_count >= bootstrap_target:
                break

        if sample_count < _BOOTSTRAP_MINIMUM_SAMPLES:
            raise RuntimeError(
                f'only {sample_count} bootstrap samples; need '
                f'{_BOOTSTRAP_MINIMUM_SAMPLES}')
        provisional_handeye, marker_pose = self._provisional_camera_model()
        current_tool = self._lookup(
            str(self.get_parameter('base_frame').value),
            str(self.get_parameter('tool_link').value))
        current_tool_position, current_tool_quaternion = _transform_tuple(
            current_tool)
        centered_candidates = generate_camera_centered_candidates(
            marker_pose[0], current_tool_position, current_tool_quaternion,
            provisional_handeye, include_fine=True)
        centered_plans, centered_stats = self._screen_candidate_plans(
            centered_candidates, optimizer)
        self.get_logger().info(
            '[CALIBRATION] camera-centred candidate probe: '
            f'ik_safe={centered_stats["ik_safe"]} '
            f'rejected_ik={centered_stats["rejected_ik"]} '
            f'rejected_condition={centered_stats["rejected_condition"]} '
            f'rejected_margin={centered_stats["rejected_margin"]} '
            f'planned_ok={centered_stats["planned_ok"]} '
            f'planned_failed={centered_stats["planned_failed"]} '
            f'total={len(centered_candidates)} tool_camera_prior=bootstrap')
        if len(centered_plans) < minimum_safe:
            raise RuntimeError(
                f'only {len(centered_plans)} safe camera-centred candidates; '
                f'need {minimum_safe}')

        # ``maximum_samples`` limits recorded samples, not motion attempts.
        # Continue through all safety-screened camera-centred plans after an
        # image rejection so the final target is constrained by measurements,
        # not by an arbitrary prefix of the plan pool.
        for plan in balanced_candidate_order(centered_plans):
            if sample_count >= maximum_samples:
                break
            candidate = plan.candidate
            if self._session_cancel.is_set() or self._session_invalid.is_set():
                raise RuntimeError('session canceled or invalidated')
            if not self._execute_candidate_plan(plan, optimizer):
                self.get_logger().warn(
                    f'[CALIBRATION] rejected during execution: {candidate.candidate_id}')
                continue
            time.sleep(float(self.get_parameter('settle_time_sec').value))
            stationary, reason = self._wait_for_joint_stationary()
            if not stationary:
                self.get_logger().warn(
                    f'[CALIBRATION] rejected {candidate.candidate_id}: '
                    f'joints did not stop: {reason}')
                continue
            frames, reason = self._stable_observation()
            if frames is None:
                self.get_logger().warn(
                    f'[CALIBRATION] image quality rejected '
                    f'{candidate.candidate_id}: {reason}')
                continue
            current_tool = self._lookup(
                str(self.get_parameter('base_frame').value),
                str(self.get_parameter('tool_link').value))
            pose = _transform_tuple(current_tool)
            if not pose_is_diverse(
                    pose[0], pose[1], accepted_poses,
                    float(self.get_parameter('minimum_translation_delta_m').value),
                    float(self.get_parameter('minimum_rotation_delta_deg').value)):
                self.get_logger().warn(
                    f'[CALIBRATION] diversity rejected {candidate.candidate_id}')
                continue
            self._publish_stable_marker_pose(frames)
            response = self._call('take', TakeSample.Request())
            new_count = len(response.samples.samples)
            if new_count <= sample_count:
                self.get_logger().warn(
                    f'[CALIBRATION] easy_handeye2 rejected {candidate.candidate_id}')
                continue
            sample_count = new_count
            accepted_poses.append(pose)
            self.get_logger().info(
                f'[CALIBRATION] sample={sample_count}/{target_samples} '
                f'candidate={candidate.candidate_id}')
            if sample_count >= target_samples:
                break

        if sample_count < minimum_samples:
            raise RuntimeError(
                f'only {sample_count} valid samples; need {minimum_samples}')
        translation_span, rotation_span = sample_coverage(accepted_poses)
        if (translation_span < float(
                self.get_parameter('minimum_translation_span_m').value)
                or rotation_span < float(
                    self.get_parameter('minimum_rotation_span_deg').value)):
            raise RuntimeError(
                f'sample coverage is insufficient: translation={translation_span:.3f}m '
                f'rotation={rotation_span:.1f}deg')
        self._solve_and_export()

    def _verify_simulation_ground_truth(self, handeye):
        """Reject a simulated solution that misses the known C10 mount."""
        if not bool(self.get_parameter('ground_truth_check_enabled').value):
            return None
        expected = (
            tuple(float(value) for value in self.get_parameter(
                'ground_truth_translation_m').value),
            tuple(float(value) for value in self.get_parameter(
                'ground_truth_rotation_xyzw').value),
        )
        translation_error, rotation_error = transform_error(handeye, expected)
        translation_limit = float(self.get_parameter(
            'ground_truth_max_translation_error_m').value)
        xy_limit = float(self.get_parameter(
            'ground_truth_max_xy_error_m').value)
        rotation_limit = float(self.get_parameter(
            'ground_truth_max_rotation_error_deg').value)
        deltas = tuple(
            float(handeye[0][index]) - float(expected[0][index])
            for index in range(3))
        passed = (translation_error <= translation_limit
                  and abs(deltas[0]) <= xy_limit
                  and abs(deltas[1]) <= xy_limit
                  and rotation_error <= rotation_limit)
        self.get_logger().info(
            '[CALIBRATION][GROUND_TRUTH] '
            f'dx={deltas[0] * 1000.0:.2f}mm '
            f'dy={deltas[1] * 1000.0:.2f}mm '
            f'dz={deltas[2] * 1000.0:.2f}mm '
            f'translation_error={translation_error * 1000.0:.2f}mm '
            f'rotation_error={rotation_error:.2f}deg '
            f'threshold_translation={translation_limit * 1000.0:.2f}mm '
            f'threshold_xy={xy_limit * 1000.0:.2f}mm '
            f'threshold_rotation={rotation_limit:.2f}deg passed={passed}')
        if not passed:
            raise RuntimeError(
                'simulation ground-truth gate failed; calibration was not saved')
        return translation_error, rotation_error

    def _solve_and_export(self):
        minimum_solution = int(
            self.get_parameter('minimum_solution_samples').value)
        position_limit = float(self.get_parameter(
            'maximum_marker_position_rms_m').value)
        rotation_limit = float(self.get_parameter(
            'maximum_marker_rotation_rms_deg').value)

        while True:
            solution = self._compute_consensus_solution()
            (selected_algorithm, handeye, samples, translation_delta,
             rotation_delta) = solution
            marker_position_rms, marker_rotation_rms = marker_pose_rms(
                samples, handeye)
            quality_ok = (
                translation_delta <= float(self.get_parameter(
                    'maximum_algorithm_translation_delta_m').value)
                and rotation_delta <= float(self.get_parameter(
                    'maximum_algorithm_rotation_delta_deg').value)
                and marker_position_rms <= position_limit
                and marker_rotation_rms <= rotation_limit)
            if quality_ok:
                break
            if len(samples) <= minimum_solution:
                raise RuntimeError(
                    'calibration quality gates failed at the minimum '
                    f'{minimum_solution}-sample subset: algorithm_spread='
                    f'{translation_delta * 1000.0:.2f}mm/'
                    f'{rotation_delta:.2f}deg marker_rms='
                    f'{marker_position_rms * 1000.0:.2f}mm/'
                    f'{marker_rotation_rms:.2f}deg')
            residuals = marker_pose_residuals(samples, handeye)
            worst_index = max(
                range(len(residuals)),
                key=lambda index: (
                    residuals[index][0] / position_limit
                    + residuals[index][1] / rotation_limit))
            worst = residuals[worst_index]
            self.get_logger().warn(
                '[CALIBRATION] rejecting outlier '
                f'index={worst_index} residual='
                f'{worst[0] * 1000.0:.2f}mm/{worst[1]:.2f}deg '
                f'samples={len(samples)}->{len(samples) - 1}')
            removed = self._call(
                'remove', RemoveSample.Request(sample_index=worst_index))
            if len(removed.samples.samples) != len(samples) - 1:
                raise RuntimeError('easy_handeye2 failed to remove outlier sample')

        retained_robot_poses = [
            transform_components(sample.robot) for sample in samples]
        translation_span, rotation_span = sample_coverage(retained_robot_poses)
        if (translation_span < float(self.get_parameter(
                'minimum_translation_span_m').value)
                or rotation_span < float(self.get_parameter(
                    'minimum_rotation_span_deg').value)):
            raise RuntimeError(
                'outlier rejection left insufficient sample coverage: '
                f'translation={translation_span:.3f}m '
                f'rotation={rotation_span:.1f}deg')
        self._verify_simulation_ground_truth(handeye)
        sample_save = self._call('save_samples', SaveSamples.Request())
        if not sample_save.success:
            raise RuntimeError('easy_handeye2 sample save failed')
        output_path = str(self.get_parameter('output_file').value)
        deployment_path = str(
            self.get_parameter('easy_handeye_output_file').value).strip()
        if deployment_path:
            deployment_output, output = write_calibration_outputs(
                handeye, deployment_path, output_path)
        else:
            deployment_output = None
            output = write_calibration(handeye, output_path)
        self.get_logger().info(
            '[CALIBRATION] SUCCESS '
            f'algorithm={selected_algorithm} samples={len(samples)} '
            f'algorithm_spread={translation_delta * 1000.0:.2f}mm/'
            f'{rotation_delta:.2f}deg marker_rms='
            f'{marker_position_rms * 1000.0:.2f}mm/'
            f'{marker_rotation_rms:.2f}deg output={output}'
            + (f' deployment={deployment_output}' if deployment_output else ''))

    def _compute_consensus_solution(self):
        """Solve all configured algorithms and select their transform medoid."""
        samples = self._call('get', TakeSample.Request()).samples.samples
        results_by_algorithm = solve_handeye(
            samples, self.get_parameter('algorithm_names').value)
        results = []
        result_by_components = {}
        for algorithm, components in results_by_algorithm.items():
            if math.sqrt(sum(value * value for value in components[0])) > float(
                self.get_parameter('maximum_camera_translation_norm_m').value):
                raise RuntimeError(
                    f'{algorithm} camera translation exceeds sanity limit')
            results.append(components)
            result_by_components[components] = algorithm
        selected, translation_delta, rotation_delta = calibration_consensus(results)
        selected_algorithm = result_by_components[selected]
        if bool(self.get_parameter('fixed_marker_refinement_enabled').value):
            refined, details = refine_handeye_fixed_marker(
                samples, selected,
                translation_sigma_m=float(self.get_parameter(
                    'fixed_marker_refinement_translation_sigma_m').value),
                rotation_sigma_deg=float(self.get_parameter(
                    'fixed_marker_refinement_rotation_sigma_deg').value),
                max_iterations=int(self.get_parameter(
                    'fixed_marker_refinement_max_iterations').value))
            self.get_logger().info(
                '[CALIBRATION] fixed-marker refinement '
                f'algorithm={selected_algorithm} '
                f'cost={details["initial_cost"]:.3f}->{details["final_cost"]:.3f} '
                f'iterations={details["iterations"]}')
            selected = refined
            selected_algorithm = f'{selected_algorithm}+fixed-marker-refine'
        return (
            selected_algorithm, selected, samples,
            translation_delta, rotation_delta)

    def destroy_node(self):
        self._request_session_stop('node shutdown')
        if bool(self.get_parameter('calibration_surface_enabled').value):
            try:
                self._arm.remove_collision_object(
                    str(self.get_parameter('calibration_surface_id').value))
            except Exception:
                pass
        return super().destroy_node()


def main():
    rclpy.init()
    node = AutoCalibrationCollector()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node._request_session_stop('Ctrl+C')
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
