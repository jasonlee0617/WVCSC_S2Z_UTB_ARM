#!/usr/bin/env python3
"""
Alicia-M Eye-to-Hand Calibration Script

Calibrates the fixed transform from an external camera to a robot arm's
base frame. The ArUco marker is attached to the robot's end-effector,
and the camera is fixed in the workspace (eye-to-hand configuration).

Uses OpenCV calibrateHandEye with inverted gripper poses to solve
for T_cam_to_base.

The gripper remains closed throughout calibration (no need to open).

Usage (conda calib):
    conda activate calib
    source /opt/ros/humble/setup.bash && source install/setup.bash
    ros2 launch alicia_m_calibration eye_to_hand_calibration.launch.py
"""

import sys
import os
import time
import threading
import numpy as np
import cv2
import yaml
from datetime import datetime
from scipy.spatial.transform import Rotation as R
from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, CameraInfo
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.srv import GetMotionPlan
from moveit_msgs.msg import Constraints, JointConstraint
from moveit_msgs.action import ExecuteTrajectory
from tf2_ros import Buffer, TransformListener
from cv_bridge import CvBridge
from rclpy.duration import Duration


class EyeToHandCalibration(Node):
    """Eye-to-hand calibration node for Alicia-M.

    The camera is fixed in the workspace and the ArUco marker is
    attached to the robot's end-effector. This solves for T_cam_to_base.
    The gripper remains closed throughout calibration.
    """

    def __init__(self):
        super().__init__('eye_to_hand_calibration')

        # Declare parameters
        self.declare_parameter('aruco_dict', 'DICT_4X4_50')
        self.declare_parameter('marker_size', 0.05)
        self.declare_parameter('marker_id', 0)
        self.declare_parameter('camera_topic', '/cam1/cam1/color/image_raw')
        self.declare_parameter('camera_info_topic', '/cam1/cam1/color/camera_info')
        self.declare_parameter('base_link', 'base_link')
        self.declare_parameter('end_effector_link', 'tool0')
        self.declare_parameter('min_samples', 15)
        self.declare_parameter('calibration_method', 'daniilidis')
        self.declare_parameter('output_file', 'eye_to_hand_cam1_result.yaml')
        self.declare_parameter('arm_id', 'master')

        # Get parameters
        self.aruco_dict_name = self.get_parameter('aruco_dict').value
        self.marker_size = self.get_parameter('marker_size').value
        self.marker_id = self.get_parameter('marker_id').value
        self.camera_topic = self.get_parameter('camera_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.base_link = self.get_parameter('base_link').value
        self.end_effector_link = self.get_parameter('end_effector_link').value
        self.min_samples = self.get_parameter('min_samples').value
        self.calibration_method = self.get_parameter('calibration_method').value
        self.output_file = self.get_parameter('output_file').value
        self.arm_id = self.get_parameter('arm_id').value

        # ArUco detector
        self._init_aruco_detector()

        # Camera state
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.camera_info_received = False
        self.current_image = None
        self.image_lock = threading.Lock()

        # ArUco detection with denoising
        self.detection_history_size = 10
        self.detection_min_frames = 5
        self.detection_rvec_history = []
        self.detection_tvec_history = []
        self.denoised_rvec = None
        self.denoised_tvec = None
        self.detection_stable = False
        self.marker_detected = False

        # Calibration samples
        # For eye-to-hand, we store INVERTED gripper poses (T_base_to_gripper)
        self.R_base2gripper_samples = []
        self.t_base2gripper_samples = []
        self.R_target2cam_samples = []
        self.t_target2cam_samples = []

        # Robot control — Alicia-M joint names (lowercase)
        self.arm_joint_names = ['joint1', 'joint2', 'joint3',
                                'joint4', 'joint5', 'joint6']
        self.arm_action_client = ActionClient(
            self, FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory',
        )
        self.execute_trajectory_client = ActionClient(
            self, ExecuteTrajectory, '/execute_trajectory',
        )
        self.plan_client = self.create_client(GetMotionPlan, '/plan_kinematic_path')

        # TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Camera subscription
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1,
        )
        self.create_subscription(Image, self.camera_topic,
                                 self._image_callback, qos)
        self.create_subscription(CameraInfo, self.camera_info_topic,
                                 self._camera_info_callback, 10)

        # Display window name (window created by first cv2.imshow call)
        self._window_name = f'Eye-to-Hand Calibration ({self.arm_id})'

        # Display timer
        self.display_timer = self.create_timer(0.033, self._display_image)

        # Calibration poses
        self.calibration_poses = self._generate_calibration_poses()

        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Eye-to-Hand Calibration ({self.arm_id})')
        self.get_logger().info(f'Camera topic: {self.camera_topic}')
        self.get_logger().info(f'ArUco: dict={self.aruco_dict_name}, '
                               f'size={self.marker_size*100:.1f}cm, id={self.marker_id}')
        self.get_logger().info(f'Output: {self.output_file}')
        self.get_logger().info('=' * 60)

    # ------------------------------------------------------------------
    # ArUco detection
    # ------------------------------------------------------------------
    def _init_aruco_detector(self):
        aruco_dict_map = {
            'DICT_4X4_50': cv2.aruco.DICT_4X4_50,
            'DICT_4X4_100': cv2.aruco.DICT_4X4_100,
            'DICT_5X5_50': cv2.aruco.DICT_5X5_50,
            'DICT_5X5_100': cv2.aruco.DICT_5X5_100,
            'DICT_6X6_50': cv2.aruco.DICT_6X6_50,
            'DICT_6X6_100': cv2.aruco.DICT_6X6_100,
        }
        if self.aruco_dict_name not in aruco_dict_map:
            self.get_logger().warn(
                f'Unknown ArUco dict: {self.aruco_dict_name}, using DICT_4X4_50')
            self.aruco_dict_name = 'DICT_4X4_50'

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(
            aruco_dict_map[self.aruco_dict_name])
        if (
            hasattr(cv2.aruco, 'DetectorParameters')
            and hasattr(cv2.aruco, 'ArucoDetector')
        ):
            self.aruco_params = cv2.aruco.DetectorParameters()
            self.aruco_detector = cv2.aruco.ArucoDetector(
                self.aruco_dict, self.aruco_params)
            self._use_new_api = True
        else:
            self.aruco_params = cv2.aruco.DetectorParameters_create()
            self.aruco_detector = None
            self._use_new_api = False

    def _detect_markers(self, gray):
        if self._use_new_api:
            return self.aruco_detector.detectMarkers(gray)
        return cv2.aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.aruco_params)

    def _image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            with self.image_lock:
                self.current_image = cv_image.copy()
                if self.camera_matrix is not None:
                    self._detect_aruco(cv_image)
        except Exception as e:
            self.get_logger().error(f'Image error: {e}')

    def _camera_info_callback(self, msg):
        if not self.camera_info_received:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.camera_info_received = True
            self.get_logger().info(f'Camera intrinsics received (fx={self.camera_matrix[0,0]:.1f})')

    def _detect_aruco(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self._detect_markers(gray)

        self.marker_detected = False
        if ids is not None:
            for i, mid in enumerate(ids.flatten()):
                if mid == self.marker_id:
                    obj_points = np.array([
                        [-self.marker_size/2, self.marker_size/2, 0],
                        [self.marker_size/2, self.marker_size/2, 0],
                        [self.marker_size/2, -self.marker_size/2, 0],
                        [-self.marker_size/2, -self.marker_size/2, 0],
                    ], dtype=np.float32)
                    ok, rvec, tvec = cv2.solvePnP(
                        obj_points, corners[i][0],
                        self.camera_matrix, self.dist_coeffs,
                    )
                    if ok:
                        self.marker_detected = True
                        self._add_detection(rvec.flatten(), tvec.flatten())
                        return
        self._add_detection(None, None)

    def _add_detection(self, rvec, tvec):
        self.detection_rvec_history.append(rvec)
        self.detection_tvec_history.append(tvec)
        if len(self.detection_rvec_history) > self.detection_history_size:
            self.detection_rvec_history.pop(0)
            self.detection_tvec_history.pop(0)
        self._compute_denoised_pose()

    def _compute_denoised_pose(self):
        valid_r = [r for r in self.detection_rvec_history if r is not None]
        valid_t = [t for t in self.detection_tvec_history if t is not None]
        if len(valid_r) < self.detection_min_frames:
            self.detection_stable = False
            self.denoised_rvec = None
            self.denoised_tvec = None
            return
        tvec_arr = np.array(valid_t)
        if np.max(np.std(tvec_arr, axis=0)) > 0.01:
            self.detection_stable = False
            self.denoised_rvec = None
            self.denoised_tvec = None
            return
        self.denoised_rvec = np.median(np.array(valid_r), axis=0)
        self.denoised_tvec = np.median(tvec_arr, axis=0)
        self.detection_stable = True

    def _clear_detection_history(self):
        self.detection_rvec_history.clear()
        self.detection_tvec_history.clear()
        self.denoised_rvec = None
        self.denoised_tvec = None
        self.detection_stable = False

    # ------------------------------------------------------------------
    # Display
    # ------------------------------------------------------------------
    def _display_image(self):
        with self.image_lock:
            if self.current_image is None:
                # Show waiting message so user knows image hasn't arrived
                blank = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(blank, f'Waiting for image on:', (30, 200),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                cv2.putText(blank, self.camera_topic, (30, 240),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 1)
                cv2.imshow(self._window_name, blank)
                cv2.waitKey(1)
                return
            display = self.current_image.copy()

        if self.camera_matrix is not None:
            gray = cv2.cvtColor(display, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = self._detect_markers(gray)
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(display, corners, ids)
                for i, mid in enumerate(ids.flatten()):
                    if mid == self.marker_id:
                        obj_pts = np.array([
                            [-self.marker_size/2, self.marker_size/2, 0],
                            [self.marker_size/2, self.marker_size/2, 0],
                            [self.marker_size/2, -self.marker_size/2, 0],
                            [-self.marker_size/2, -self.marker_size/2, 0],
                        ], dtype=np.float32)
                        ok, rvec, tvec = cv2.solvePnP(
                            obj_pts, corners[i][0],
                            self.camera_matrix, self.dist_coeffs)
                        if ok:
                            cv2.drawFrameAxes(display, self.camera_matrix,
                                              self.dist_coeffs, rvec, tvec,
                                              self.marker_size * 0.5)
                            distance = np.linalg.norm(tvec)
                            cv2.putText(display, f'Distance: {distance:.3f}m',
                                        (10, 150), cv2.FONT_HERSHEY_SIMPLEX,
                                        0.7, (0, 255, 0), 2)

        color = (0, 255, 0) if self.marker_detected else (0, 0, 255)
        cv2.putText(display, f'Marker {self.marker_id}: '
                    f'{"Detected" if self.marker_detected else "Not Found"}',
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        n = len(self.R_base2gripper_samples)
        cv2.putText(display, f'Samples: {n}/{self.min_samples}',
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        stable_c = (0, 255, 0) if self.detection_stable else (0, 165, 255)
        valid_count = sum(1 for r in self.detection_rvec_history if r is not None)
        cv2.putText(display, f'Stable: {"Yes" if self.detection_stable else "No"} '
                    f'({valid_count}/{self.detection_history_size})',
                    (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, stable_c, 2)
        cv2.putText(display, f'Eye-to-Hand ({self.arm_id})',
                    (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2)

        cv2.imshow(self._window_name, display)
        cv2.waitKey(1)

    # ------------------------------------------------------------------
    # Calibration poses for eye-to-hand
    # ------------------------------------------------------------------
    def _generate_calibration_poses(self):
        """Generate diverse poses for eye-to-hand calibration.

        The marker is on the gripper and must be visible to the overhead
        camera. Poses are small perturbations around arm-specific reference
        positions where the ArUco marker faces the camera.

        Reference positions (camera overhead, gripper parallel to table):
          master (cam1): [+0.4671, -1.0748, -0.2794, +0.2691, +0.9573, -0.2024]
          slave  (cam2): [-0.2882, -1.2366, -0.4435, -0.1459, +0.8368, +0.1047]
        """
        # Select reference pose based on arm_id
        if self.arm_id == 'slave':
            ref = [-0.38, -1.34, -0.52, -0.24, 1.05, +0.02]
            # ref = [-0.2882, -1.2366, -0.4435, -0.1459, +0.8368, +0.1047]
        else:  # master (default)
            ref = [+0.37, -1.18, -0.34, +0.15, +0.99, -0.11]
            # ref = [+0.4671, -1.0748, -0.2794, +0.2691, +0.9573, -0.2024]

        def p(d1=0, d2=0, d3=0, d4=0, d5=0, d6=0):
            return [ref[0]+d1, ref[1]+d2, ref[2]+d3,
                    ref[3]+d4, ref[4]+d5, ref[5]+d6]

        poses = []
        # Reference pose
        poses.append(p())

        # Vary J1 (base rotation) - small range
        poses.append(p(d1=+0.08))
        poses.append(p(d1=-0.08))
        poses.append(p(d1=+0.15))
        poses.append(p(d1=-0.15))

        # Vary J2 (shoulder)
        poses.append(p(d2=+0.06))
        poses.append(p(d2=-0.06))
        poses.append(p(d2=+0.12))

        # Vary J3 (elbow)
        poses.append(p(d3=+0.06))
        poses.append(p(d3=-0.06))

        # Vary J4 (wrist rotation)
        poses.append(p(d4=+0.10))
        poses.append(p(d4=-0.10))

        # Vary J5 (wrist tilt)
        poses.append(p(d5=-0.10))
        poses.append(p(d5=+0.10))

        # Vary J6 (flange rotation)
        poses.append(p(d6=+0.20))
        poses.append(p(d6=-0.20))
        poses.append(p(d6=+0.35))

        # Combined: J1 + J2
        poses.append(p(d1=+0.08, d2=+0.06))
        poses.append(p(d1=-0.08, d2=-0.06))

        # Combined: J1 + J5 + J6
        poses.append(p(d1=+0.08, d5=-0.08, d6=+0.20))
        poses.append(p(d1=-0.08, d5=+0.08, d6=-0.20))

        # Combined: J2 + J3
        poses.append(p(d2=+0.06, d3=-0.06))
        poses.append(p(d2=-0.06, d3=+0.06))

        return poses

    # ------------------------------------------------------------------
    # Robot motion control
    # ------------------------------------------------------------------
    def wait_for_services(self):
        self.get_logger().info('Waiting for robot services...')
        if not self.arm_action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('FollowJointTrajectory server unavailable')
            return False
        if not self.execute_trajectory_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('ExecuteTrajectory server unavailable')
            return False
        if not self.plan_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('Motion planning service unavailable')
            return False

        self.get_logger().info('Waiting for camera data...')
        t0 = time.time()
        while not self.camera_info_received:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - t0 > 30.0:
                self.get_logger().error('Camera data timeout')
                return False

        self.get_logger().info('All services ready')
        return True

    def plan_to_joint_positions(self, joint_positions):
        request = GetMotionPlan.Request()
        request.motion_plan_request.group_name = "arm"
        request.motion_plan_request.num_planning_attempts = 5
        request.motion_plan_request.allowed_planning_time = 5.0
        request.motion_plan_request.max_velocity_scaling_factor = 0.3
        request.motion_plan_request.max_acceleration_scaling_factor = 0.3
        request.motion_plan_request.start_state.is_diff = True

        goal_constraints = Constraints()
        goal_constraints.name = "joint_goal"
        for i, name in enumerate(self.arm_joint_names):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(joint_positions[i])
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            goal_constraints.joint_constraints.append(jc)
        request.motion_plan_request.goal_constraints.append(goal_constraints)

        try:
            future = self.plan_client.call_async(request)
            t0 = time.time()
            while not future.done():
                rclpy.spin_once(self, timeout_sec=0.05)
                if time.time() - t0 > 10.0:
                    return None
            resp = future.result()
            if resp.motion_plan_response.error_code.val == 1:
                return resp.motion_plan_response.trajectory
        except Exception as e:
            self.get_logger().error(f'Planning failed: {e}')
        return None

    def execute_trajectory(self, trajectory):
        goal_msg = ExecuteTrajectory.Goal()
        goal_msg.trajectory = trajectory
        future = self.execute_trajectory_client.send_goal_async(goal_msg)
        t0 = time.time()
        while not future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.time() - t0 > 10.0:
                return False
        handle = future.result()
        if not handle.accepted:
            return False

        if trajectory.joint_trajectory.points:
            last_pt = trajectory.joint_trajectory.points[-1]
            dur = last_pt.time_from_start.sec + last_pt.time_from_start.nanosec / 1e9
            timeout = dur + 15.0
        else:
            timeout = 30.0

        result_future = handle.get_result_async()
        t0 = time.time()
        while not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.time() - t0 > timeout:
                return False
        result = result_future.result()
        return result.result.error_code.val == 1

    def move_to_pose(self, joint_positions, duration_sec=4.0):
        traj = self.plan_to_joint_positions(joint_positions)
        if traj is not None and self.execute_trajectory(traj):
            time.sleep(0.8)
            # time.sleep(0.5)
            return True
        # Fallback: direct trajectory
        return self._move_direct(joint_positions, duration_sec)

    def _move_direct(self, joint_positions, duration_sec=4.0):
        goal = FollowJointTrajectory.Goal()
        traj = JointTrajectory()
        traj.joint_names = self.arm_joint_names
        pt = JointTrajectoryPoint()
        pt.positions = joint_positions
        pt.time_from_start = Duration(seconds=duration_sec).to_msg()
        traj.points.append(pt)
        goal.trajectory = traj

        future = self.arm_action_client.send_goal_async(goal)
        t0 = time.time()
        while not future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.time() - t0 > 10.0:
                return False
        handle = future.result()
        if not handle.accepted:
            return False
        result_future = handle.get_result_async()
        t0 = time.time()
        while not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.time() - t0 > duration_sec + 10.0:
                return False
        return result_future.result().result.error_code == 0

    # ------------------------------------------------------------------
    # Pose acquisition
    # ------------------------------------------------------------------
    def get_end_effector_pose(self):
        """Get T_gripper_to_base from TF, then invert to T_base_to_gripper."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_link, self.end_effector_link,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0),
            )
            q = tf.transform.rotation
            t = tf.transform.translation
            R_g2b = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
            t_g2b = np.array([t.x, t.y, t.z])

            # Invert: T_base_to_gripper = inv(T_gripper_to_base)
            R_b2g = R_g2b.T
            t_b2g = -R_b2g @ t_g2b
            return R_b2g, t_b2g
        except Exception as e:
            self.get_logger().error(f'TF lookup failed: {e}')
            return None

    def get_marker_pose(self):
        """Get T_marker_to_cam from denoised ArUco detection."""
        if self.detection_stable and self.denoised_rvec is not None:
            R_mat, _ = cv2.Rodrigues(self.denoised_rvec)
            return R_mat, self.denoised_tvec
        if self.marker_detected:
            rvec = self.detection_rvec_history[-1]
            tvec = self.detection_tvec_history[-1]
            if rvec is not None:
                R_mat, _ = cv2.Rodrigues(rvec)
                return R_mat, tvec
        return None

    def record_sample(self):
        """Record one calibration sample pair."""
        ee_result = self.get_end_effector_pose()
        if ee_result is None:
            self.get_logger().warn('Cannot get end-effector pose')
            return False

        marker_result = self.get_marker_pose()
        if marker_result is None:
            self.get_logger().warn('Cannot get marker pose')
            return False

        R_b2g, t_b2g = ee_result
        R_t2c, t_t2c = marker_result

        self.R_base2gripper_samples.append(R_b2g)
        self.t_base2gripper_samples.append(t_b2g)
        self.R_target2cam_samples.append(R_t2c)
        self.t_target2cam_samples.append(t_t2c)

        n = len(self.R_base2gripper_samples)
        self.get_logger().info(f'Sample #{n} recorded')
        return True

    # ------------------------------------------------------------------
    # Calibration computation
    # ------------------------------------------------------------------
    def compute_calibration(self):
        """Compute eye-to-hand calibration: T_cam_to_base."""
        n = len(self.R_base2gripper_samples)
        if n < 3:
            self.get_logger().error(f'Not enough samples: {n} (need >= 3)')
            return None

        method_map = {
            'tsai': cv2.CALIB_HAND_EYE_TSAI,
            'park': cv2.CALIB_HAND_EYE_PARK,
            'horaud': cv2.CALIB_HAND_EYE_HORAUD,
            'daniilidis': cv2.CALIB_HAND_EYE_DANIILIDIS,
            'andreff': cv2.CALIB_HAND_EYE_ANDREFF,
        }
        if self.calibration_method not in method_map:
            self.calibration_method = 'daniilidis'
        method = method_map[self.calibration_method]

        R_g = [r for r in self.R_base2gripper_samples]
        t_g = [t.reshape(3, 1) for t in self.t_base2gripper_samples]
        R_t = [r for r in self.R_target2cam_samples]
        t_t = [t.reshape(3, 1) for t in self.t_target2cam_samples]

        try:
            R_cam2base, t_cam2base = cv2.calibrateHandEye(
                R_g, t_g, R_t, t_t, method=method,
            )
            self.get_logger().info(f'Calibration computed ({self.calibration_method})')
            return R_cam2base, t_cam2base.flatten()
        except Exception as e:
            self.get_logger().error(f'Calibration failed: {e}')
            return None

    def compute_with_multiple_methods(self):
        """Run all calibration methods and compare."""
        methods = {
            'tsai': cv2.CALIB_HAND_EYE_TSAI,
            'park': cv2.CALIB_HAND_EYE_PARK,
            'horaud': cv2.CALIB_HAND_EYE_HORAUD,
            'daniilidis': cv2.CALIB_HAND_EYE_DANIILIDIS,
            'andreff': cv2.CALIB_HAND_EYE_ANDREFF,
        }
        R_g = [r for r in self.R_base2gripper_samples]
        t_g = [t.reshape(3, 1) for t in self.t_base2gripper_samples]
        R_t = [r for r in self.R_target2cam_samples]
        t_t = [t.reshape(3, 1) for t in self.t_target2cam_samples]

        results = {}
        for name, method in methods.items():
            try:
                rot, trans = cv2.calibrateHandEye(
                    R_g, t_g, R_t, t_t, method=method)
                results[name] = {'R': rot, 't': trans.flatten(), 'success': True}
            except Exception as e:
                results[name] = {'error': str(e), 'success': False}
        return results

    # ------------------------------------------------------------------
    # Save result
    # ------------------------------------------------------------------
    def save_result(self, R_cam2base, t_cam2base):
        """Save calibration result to YAML."""
        rot = R.from_matrix(R_cam2base)
        quat = rot.as_quat()  # [x, y, z, w]
        euler = rot.as_euler('xyz', degrees=True)

        T = np.eye(4)
        T[:3, :3] = R_cam2base
        T[:3, 3] = t_cam2base

        result = {
            'eye_to_hand_calibration': {
                'type': 'eye_to_hand',
                'arm_id': self.arm_id,
                'frame_id': self.base_link,
                'child_frame_id': 'camera_link',
                'date': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                'samples_count': len(self.R_base2gripper_samples),
                'calibration_method': self.calibration_method,
                'aruco_marker_id': self.marker_id,
                'aruco_marker_size': self.marker_size,
                'transform': {
                    'translation': {
                        'x': float(t_cam2base[0]),
                        'y': float(t_cam2base[1]),
                        'z': float(t_cam2base[2]),
                    },
                    'rotation': {
                        'quaternion': {
                            'x': float(quat[0]),
                            'y': float(quat[1]),
                            'z': float(quat[2]),
                            'w': float(quat[3]),
                        },
                        'euler_xyz_deg': {
                            'roll': float(euler[0]),
                            'pitch': float(euler[1]),
                            'yaw': float(euler[2]),
                        },
                    },
                    'matrix_4x4': T.tolist(),
                },
            }
        }

        # Find config directory
        try:
            pkg_share = get_package_share_directory('alicia_m_calibration')
            ws_root = os.path.abspath(os.path.join(pkg_share, '..', '..', '..', '..'))
            config_dir = os.path.join(ws_root, 'src', 'alicia_m_calibration', 'config')
        except Exception:
            config_dir = os.path.join(os.path.dirname(__file__), '..', 'config')

        os.makedirs(config_dir, exist_ok=True)
        path = os.path.join(config_dir, self.output_file)

        with open(path, 'w') as f:
            yaml.dump(result, f, default_flow_style=False, allow_unicode=True)

        abs_path = os.path.abspath(path)
        self.get_logger().info(f'Result saved to: {abs_path}')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Eye-to-Hand Calibration Result (T_cam_to_{self.arm_id}_base)')
        self.get_logger().info(f'Translation (m): x={t_cam2base[0]:.6f}, '
                               f'y={t_cam2base[1]:.6f}, z={t_cam2base[2]:.6f}')
        self.get_logger().info(f'Quaternion: x={quat[0]:.6f}, y={quat[1]:.6f}, '
                               f'z={quat[2]:.6f}, w={quat[3]:.6f}')
        self.get_logger().info(f'Euler (deg): roll={euler[0]:.2f}, '
                               f'pitch={euler[1]:.2f}, yaw={euler[2]:.2f}')
        self.get_logger().info('=' * 60)
        return path

    # ------------------------------------------------------------------
    # Main calibration flow
    # ------------------------------------------------------------------
    def run_calibration(self):
        self.get_logger().info('Starting eye-to-hand calibration')

        if not self.wait_for_services():
            return False

        # Move to first calibration pose (gripper must be closed manually beforehand)
        self.get_logger().info('NOTE: Gripper should already be closed with ArUco marker attached')
        ref_pose = self.calibration_poses[0]
        self.get_logger().info(f'Moving to reference pose...')
        if not self.move_to_pose(ref_pose, 5.0):
            self.get_logger().error('Cannot reach reference pose')
            return False
        time.sleep(1.0)

        # Iterate calibration poses
        pose_idx = 0
        while (len(self.R_base2gripper_samples) < self.min_samples
               and pose_idx < len(self.calibration_poses)):
            self.get_logger().info(
                f'--- Pose {pose_idx+1}/{len(self.calibration_poses)} ---')

            if not self.move_to_pose(self.calibration_poses[pose_idx], 4.0):
                self.get_logger().warn(f'Pose {pose_idx+1} failed, skipping')
                pose_idx += 1
                continue

            # Wait for arm to settle
            t0 = time.time()
            while time.time() - t0 < 2.0:
                rclpy.spin_once(self, timeout_sec=0.05)

            # Collect stable detection
            self._clear_detection_history()
            t0 = time.time()
            while time.time() - t0 < 5.0:
                rclpy.spin_once(self, timeout_sec=0.05)
                if self.detection_stable:
                    break

            if not self.detection_stable:
                self.get_logger().warn(f'Pose {pose_idx+1}: detection unstable')
                pose_idx += 1
                continue

            if self.record_sample():
                n = len(self.R_base2gripper_samples)
                self.get_logger().info(f'Recorded {n}/{self.min_samples}')
            pose_idx += 1

        # Return to reference pose
        self.move_to_pose(ref_pose, 5.0)
        # self.move_to_pose(ref_pose, 3.0)

        n = len(self.R_base2gripper_samples)
        if n < 3:
            self.get_logger().error(f'Not enough samples: {n}')
            return False

        if n < self.min_samples:
            self.get_logger().warn(f'Samples below target: {n}/{self.min_samples}')

        # Compute with all methods
        self.get_logger().info('Computing calibration with all methods...')
        all_results = self.compute_with_multiple_methods()
        for name, res in all_results.items():
            if res['success']:
                t = res['t']
                self.get_logger().info(
                    f'  {name}: t=[{t[0]:.4f}, {t[1]:.4f}, {t[2]:.4f}]')
            else:
                self.get_logger().warn(f'  {name}: failed - {res.get("error")}')

        # Use selected method
        if (self.calibration_method in all_results
                and all_results[self.calibration_method]['success']):
            R_res = all_results[self.calibration_method]['R']
            t_res = all_results[self.calibration_method]['t']
        else:
            for name, res in all_results.items():
                if res['success']:
                    R_res = res['R']
                    t_res = res['t']
                    self.calibration_method = name
                    break
            else:
                self.get_logger().error('All methods failed')
                return False

        self.save_result(R_res, t_res)
        self.get_logger().info('Eye-to-hand calibration complete!')
        return True

    def shutdown(self):
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = EyeToHandCalibration()
    success = False
    try:
        success = node.run_calibration()
    except KeyboardInterrupt:
        print('\nCalibration interrupted')
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()
    return 0 if success else 1


if __name__ == '__main__':
    sys.exit(main())