#!/usr/bin/env python3
"""Interactive, Alicia-M-adaptive eye-in-hand calibration collector.

The launch terminal owns hardware, MoveIt, C10, ArUco and easy_handeye2.  This
second-terminal process owns only candidate generation and sample collection.
Press ``s`` or Enter to start one fresh session and ``q`` to cancel it.  Any
external stop/reset event invalidates the whole session; after HOME and resume,
the operator must press ``s`` again.
"""

from collections import deque
import math
import os
import select
import sys
import threading
import time
from types import SimpleNamespace

import cv2
from cv_bridge import CvBridge
from easy_handeye2_msgs.srv import (
    ComputeCalibration,
    RemoveSample,
    SaveCalibration,
    SaveSamples,
    SetAlgorithm,
    TakeSample,
)
import numpy as np
import rclpy
from rcl_interfaces.srv import GetParameters
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener

from wvcsc_arm_task.motion_state import MotionControlState
from wvcsc_arm_task.node_parameters import create_alicia_moveit
from wvcsc_arm_task.observation import (
    ObservationOptimizer,
    rotate_vector,
    tool_pose_from_camera_pose,
)

from .alicia_sample_geometry import generate_alicia_candidates
from .calibration_io import export_calibration
from .calibration_quality import (
    MarkerObservation,
    calibration_consensus,
    marker_pose_rms,
    marker_pose_residuals,
    pose_is_diverse,
    sample_coverage,
    stable_marker_window,
    transform_components,
)


def _wait_future(future, timeout_sec):
    deadline = time.monotonic() + float(timeout_sec)
    while rclpy.ok() and not future.done() and time.monotonic() < deadline:
        time.sleep(0.01)
    return future.done()


def _transform_tuple(stamped):
    transform = stamped.transform
    return transform_components(transform)


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
        self._external_locked = False
        self._emergency_stop = False
        self._joint_positions = None
        self._camera_info = None
        self._observations = deque(maxlen=max(
            2 * int(self.get_parameter('stable_frames').value), 20))

        self._dictionary = self._create_dictionary()
        self._detector_parameters = (
            cv2.aruco.DetectorParameters()
            if hasattr(cv2.aruco, 'DetectorParameters')
            else cv2.aruco.DetectorParameters_create())
        self._clients = self._create_easy_clients()
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            JointState, str(self.get_parameter('joint_state_topic').value),
            self._on_joint_state, 10, callback_group=self._group)
        self.create_subscription(
            CameraInfo, str(self.get_parameter('camera_info_topic').value),
            self._on_camera_info, 10, callback_group=self._group)
        self.create_subscription(
            Image, str(self.get_parameter('image_topic').value),
            self._on_image, 10, callback_group=self._group)
        self.create_subscription(
            Bool, str(self.get_parameter('motion_locked_topic').value),
            self._on_motion_locked, latched, callback_group=self._group)
        self.create_subscription(
            String, str(self.get_parameter('motion_state_topic').value),
            self._on_motion_state, latched, callback_group=self._group)
        self.create_subscription(
            Bool, str(self.get_parameter('emergency_stop_topic').value),
            self._on_emergency_stop, latched, callback_group=self._group)

        self._keyboard_thread = threading.Thread(
            target=self._keyboard_loop, daemon=True)
        self._keyboard_thread.start()
        self.get_logger().info(
            '[CALIBRATION] Alicia-M collector ready: s/Enter=start, q=cancel. '
            'Use motion_control_keyboard for SPACE/h/r/x safety control.')

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
            'emergency_stop_topic': '/safety/emergency_stop',
            'minimum_samples': 15,
            'minimum_solution_samples': 14,
            'target_samples': 18,
            'maximum_samples': 22,
            'minimum_safe_candidates': 14,
            'stable_frames': 10,
            'settle_time_sec': 1.0,
            'marker_distance_min_m': 0.25,
            'marker_distance_max_m': 0.80,
            'minimum_corner_margin_px': 60.0,
            'maximum_center_std_px': 4.0,
            'maximum_marker_depth_std_m': 0.003,
            'maximum_marker_angle_std_deg': 0.8,
            'minimum_translation_delta_m': 0.006,
            'minimum_rotation_delta_deg': 3.0,
            'minimum_translation_span_m': 0.04,
            'minimum_rotation_span_deg': 20.0,
            'maximum_center_error_px': 45.0,
            'recenter_step_limit_m': 0.003,
            'recenter_total_limit_m': 0.010,
            'recenter_attempt_limit': 3,
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
            'easy_service_timeout_sec': 10.0,
            'output_file': '~/.ros/wvcsc_calibration/c10_handeye.yaml',
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
            'set_algorithm': (
                SetAlgorithm, '/easy_handeye2/calibration/set_algorithm'),
            'compute': (
                ComputeCalibration, '/easy_handeye2/calibration/compute_calibration'),
            'save': (
                SaveCalibration, '/easy_handeye2/calibration/save_calibration'),
            'save_samples': (
                SaveSamples, '/easy_handeye2/calibration/save_samples'),
        }
        return {
            name: self.create_client(interface, service, callback_group=self._group)
            for name, (interface, service) in services.items()
        }

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
            marker_corners = np.asarray(corners[index], dtype=float).reshape(4, 2)
            camera, distortion, width, height = camera_info
            rvecs, tvecs, _objects = cv2.aruco.estimatePoseSingleMarkers(
                np.asarray([marker_corners], dtype=np.float32),
                float(self.get_parameter('marker_size_m').value),
                camera, distortion)
            centre = np.mean(marker_corners, axis=0)
            margin = min(
                float(np.min(marker_corners[:, 0])),
                float(np.min(marker_corners[:, 1])),
                float(width - np.max(marker_corners[:, 0])),
                float(height - np.max(marker_corners[:, 1])))
            observation = MarkerObservation(
                center_px=(float(centre[0]), float(centre[1])),
                margin_px=margin,
                translation=tuple(float(value) for value in tvecs[0].reshape(3)),
                rotation_vector=tuple(float(value) for value in rvecs[0].reshape(3)),
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
            f'[CALIBRATION] session invalidated by safety state: {reason}')

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
                'HOME_LOCKED', 'HARD_STOPPED'}:
            self._invalidate_session(state)
        elif state in {'NORMAL', 'RUNNING', 'READY'} and not self._emergency_stop:
            self._state.resume()

    def _on_emergency_stop(self, message):
        self._emergency_stop = bool(message.data)
        self._state.set_hard_stop(self._emergency_stop)
        if self._emergency_stop:
            self._invalidate_session('physical emergency stop')

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

    def _start_session(self):
        with self._session_lock:
            if self._session_thread is not None and self._session_thread.is_alive():
                self.get_logger().warn('[CALIBRATION] a collection session is active')
                return
            if self._external_locked or self._emergency_stop or self._state.locked:
                self.get_logger().error(
                    '[CALIBRATION] motion is locked; complete h -> HOME_LOCKED -> r first')
                return
            self._session_cancel.clear()
            self._session_invalid.clear()
            self._session_thread = threading.Thread(
                target=self._run_session_guarded, daemon=True)
            self._session_thread.start()

    def _request_session_stop(self, reason):
        self.get_logger().warn(f'[CALIBRATION] cancel requested: {reason}')
        self._session_cancel.set()
        self._arm.cancel()

    def _run_session_guarded(self):
        seed = None
        try:
            # 先等待完整输入再固定会话起始关节。若操作员在节点刚启动、
            # 第一帧 joint_states 到达前按下 s，仍必须能够在 q/结束后回到
            # 真实起始姿态，而不是因为 seed=None 跳过安全回退。
            initial_inputs = self._wait_inputs()
            seed = initial_inputs[0]
            self._run_session(initial_inputs)
        except Exception as error:
            self.get_logger().error(f'[CALIBRATION] session failed: {error}')
        finally:
            if (seed is not None and not self._session_invalid.is_set()
                    and not self._state.locked and not self._emergency_stop):
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
                        str(self.get_parameter('base_frame').value),
                        str(self.get_parameter('camera_frame').value)),
                    self._lookup(
                        str(self.get_parameter('base_frame').value),
                        str(self.get_parameter('marker_frame').value)),
                    self._lookup(
                        str(self.get_parameter('tool_link').value),
                        str(self.get_parameter('camera_frame').value)),
                )
            except TransformException:
                transforms = None
            if joints is not None and camera is not None and transforms is not None:
                return joints, camera, transforms
            time.sleep(0.05)
        raise RuntimeError('joint state, CameraInfo or calibration TF is unavailable')

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

    def _safe_plan(self, candidate, optimizer):
        joints = self._current_joints()
        if joints is None:
            return None
        solution = self._arm.compute_ik(
            candidate.tool_position, candidate.tool_quaternion, joints, timeout=0.5)
        if solution is None:
            return None
        by_name = dict(zip(solution.name, solution.position))
        names = tuple(str(value) for value in self.get_parameter('joint_names').value)
        try:
            ik_joints = tuple(float(by_name[name]) for name in names)
        except KeyError:
            return None
        if (optimizer.condition_number(ik_joints) >= float(
                self.get_parameter('max_condition_number').value)
                or optimizer.minimum_joint_margin(ik_joints) < float(
                    self.get_parameter('min_joint_margin_rad').value)):
            return None
        trajectory = self._arm.plan_pose(
            candidate.tool_position, candidate.tool_quaternion,
            frame_id=str(self.get_parameter('base_frame').value),
            tolerance_position=float(
                self.get_parameter('position_tolerance_m').value),
            tolerance_orientation=float(
                self.get_parameter('orientation_tolerance_rad').value))
        final = self._arm.trajectory_final_positions(trajectory, names)
        if (final is None
                or optimizer.condition_number(final) >= float(
                    self.get_parameter('max_condition_number').value)
                or optimizer.minimum_joint_margin(final) < float(
                    self.get_parameter('min_joint_margin_rad').value)):
            return None
        return trajectory

    def _stable_observation(self, timeout_sec=5.0):
        with self._data_lock:
            self._observations.clear()
        deadline = time.monotonic() + timeout_sec
        required = int(self.get_parameter('stable_frames').value)
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
                minimum_margin_px=float(
                    self.get_parameter('minimum_corner_margin_px').value),
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

    def _recenter_marker_if_needed(self, frames, optimizer, camera_mount):
        """Apply at most three collision-checked 3 mm image-plane corrections."""
        total_motion = 0.0
        attempts = int(self.get_parameter('recenter_attempt_limit').value)
        maximum_error = float(
            self.get_parameter('maximum_center_error_px').value)
        step_limit = float(self.get_parameter('recenter_step_limit_m').value)
        total_limit = float(self.get_parameter('recenter_total_limit_m').value)
        for _index in range(attempts + 1):
            with self._data_lock:
                camera_info = self._camera_info
            if camera_info is None or not frames:
                return None
            matrix = camera_info[0]
            observation = frames[-1]
            error_u = observation.center_px[0] - float(matrix[0, 2])
            error_v = observation.center_px[1] - float(matrix[1, 2])
            if math.hypot(error_u, error_v) <= maximum_error:
                return frames
            if _index >= attempts:
                return None
            depth = float(observation.translation[2])
            camera_delta = np.asarray((
                error_u * depth / float(matrix[0, 0]),
                error_v * depth / float(matrix[1, 1]),
                0.0,
            ), dtype=float)
            norm = float(np.linalg.norm(camera_delta))
            if norm <= 1.0e-9:
                return None
            camera_delta *= min(1.0, step_limit / norm)
            step = float(np.linalg.norm(camera_delta))
            if total_motion + step > total_limit + 1.0e-12:
                return None
            base_camera = self._lookup(
                str(self.get_parameter('base_frame').value),
                str(self.get_parameter('camera_frame').value))
            camera_position, camera_quaternion = _transform_tuple(base_camera)
            base_delta = rotate_vector(tuple(camera_delta), camera_quaternion)
            corrected_camera_position = tuple(
                camera_position[index] + base_delta[index] for index in range(3))
            tool_position, tool_quaternion = tool_pose_from_camera_pose(
                corrected_camera_position, camera_quaternion,
                camera_mount[0], camera_mount[1])
            candidate = SimpleNamespace(
                tool_position=tool_position,
                tool_quaternion=tool_quaternion)
            trajectory = self._safe_plan(candidate, optimizer)
            if trajectory is None or not self._arm.execute_trajectory(trajectory):
                return None
            total_motion += step
            time.sleep(float(self.get_parameter('settle_time_sec').value))
            frames, _reason = self._stable_observation()
            if frames is None:
                return None
        return None

    def _call(self, name, request):
        client = self._clients[name]
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

    def _run_session(self, initial_inputs=None):
        _joints, _camera, transforms = (
            self._wait_inputs() if initial_inputs is None else initial_inputs)
        self._clear_easy_samples()
        optimizer = self._optimizer()
        _base_tool, base_camera, base_marker, tool_camera = transforms
        marker_position, _marker_quaternion = _transform_tuple(base_marker)
        camera_position, camera_quaternion = _transform_tuple(base_camera)
        mount_translation, mount_quaternion = _transform_tuple(tool_camera)
        candidates = generate_alicia_candidates(
            marker_position, camera_position, camera_quaternion,
            mount_translation, mount_quaternion)
        safe = [candidate for candidate in candidates
                if self._safe_plan(candidate, optimizer) is not None]
        minimum_safe = int(self.get_parameter('minimum_safe_candidates').value)
        if len(safe) < minimum_safe:
            raise RuntimeError(
                f'only {len(safe)} safe Alicia-M candidates; need {minimum_safe}')
        self.get_logger().info(
            f'[CALIBRATION] {len(safe)}/{len(candidates)} candidates passed '
            'collision IK, Jacobian, joint-margin and OMPL checks')

        accepted_poses = []
        sample_count = 0
        target_samples = int(self.get_parameter('target_samples').value)
        maximum_samples = int(self.get_parameter('maximum_samples').value)
        for candidate in safe[:maximum_samples]:
            if self._session_cancel.is_set() or self._session_invalid.is_set():
                raise RuntimeError('session canceled or invalidated')
            trajectory = self._safe_plan(candidate, optimizer)
            if trajectory is None or not self._arm.execute_trajectory(trajectory):
                self.get_logger().warn(
                    f'[CALIBRATION] rejected during execution: {candidate.candidate_id}')
                continue
            time.sleep(float(self.get_parameter('settle_time_sec').value))
            frames, reason = self._stable_observation()
            if frames is None:
                self.get_logger().warn(
                    f'[CALIBRATION] image quality rejected '
                    f'{candidate.candidate_id}: {reason}')
                continue
            frames = self._recenter_marker_if_needed(
                frames, optimizer, (mount_translation, mount_quaternion))
            if frames is None:
                self.get_logger().warn(
                    f'[CALIBRATION] marker recenter rejected '
                    f'{candidate.candidate_id}')
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

        minimum_samples = int(self.get_parameter('minimum_samples').value)
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

    def _solve_and_export(self):
        minimum_solution = int(
            self.get_parameter('minimum_solution_samples').value)
        position_limit = float(self.get_parameter(
            'maximum_marker_position_rms_m').value)
        rotation_limit = float(self.get_parameter(
            'maximum_marker_rotation_rms_deg').value)

        while True:
            solution = self._compute_consensus_solution()
            (selected_algorithm, computed, samples, translation_delta,
             rotation_delta) = solution
            handeye = transform_components(computed.calibration.transform)
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
        saved = self._call('save', SaveCalibration.Request())
        if not saved.success:
            raise RuntimeError('easy_handeye2 calibration save failed')
        sample_save = self._call('save_samples', SaveSamples.Request())
        if not sample_save.success:
            raise RuntimeError('easy_handeye2 sample save failed')
        output = export_calibration(
            saved.filepath.data,
            os.path.expanduser(str(self.get_parameter('output_file').value)))
        self.get_logger().info(
            '[CALIBRATION] SUCCESS '
            f'algorithm={selected_algorithm} samples={len(samples)} '
            f'algorithm_spread={translation_delta * 1000.0:.2f}mm/'
            f'{rotation_delta:.2f}deg marker_rms='
            f'{marker_position_rms * 1000.0:.2f}mm/'
            f'{marker_rotation_rms:.2f}deg output={output}')

    def _compute_consensus_solution(self):
        """Solve all configured algorithms and select their transform medoid."""
        results = []
        result_by_components = {}
        for algorithm in self.get_parameter('algorithm_names').value:
            algorithm = str(algorithm)
            set_response = self._call(
                'set_algorithm', SetAlgorithm.Request(new_algorithm=algorithm))
            if not set_response.success:
                raise RuntimeError(f'algorithm unavailable: {algorithm}')
            computed = self._call('compute', ComputeCalibration.Request())
            if not computed.valid:
                raise RuntimeError(f'calibration failed: {algorithm}')
            components = transform_components(computed.calibration.transform)
            if math.sqrt(sum(value * value for value in components[0])) > float(
                    self.get_parameter('maximum_camera_translation_norm_m').value):
                raise RuntimeError(
                    f'{algorithm} camera translation exceeds sanity limit')
            results.append(components)
            result_by_components[components] = algorithm
        selected, translation_delta, rotation_delta = calibration_consensus(results)
        selected_algorithm = result_by_components[selected]
        self._call(
            'set_algorithm', SetAlgorithm.Request(new_algorithm=selected_algorithm))
        computed = self._call('compute', ComputeCalibration.Request())
        samples = self._call('get', TakeSample.Request()).samples.samples
        return (
            selected_algorithm, computed, samples,
            translation_delta, rotation_delta)

    def destroy_node(self):
        self._request_session_stop('node shutdown')
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
