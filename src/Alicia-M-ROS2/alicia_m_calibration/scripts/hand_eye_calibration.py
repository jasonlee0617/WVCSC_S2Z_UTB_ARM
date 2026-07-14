#!/usr/bin/env python3
"""
Alicia-M 机械臂手眼标定脚本 (Eye-in-Hand)

使用 ArUco 标记和 RealSense D405 相机进行手眼标定。
标定完成后，输出机械臂末端(tool0)到相机的变换矩阵，保存在../config/hand_eye_calibration_result.yaml

TF树连接: tool0 -> camera_link -> ... -> camera_color_optical_frame -> aruco_marker
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
from geometry_msgs.msg import TransformStamped
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.srv import GetMotionPlan
from moveit_msgs.msg import Constraints, JointConstraint
from moveit_msgs.action import ExecuteTrajectory
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from cv_bridge import CvBridge
from rclpy.duration import Duration


class HandEyeCalibration(Node):
    """手眼标定节点 (Eye-in-Hand 配置) - Alicia-M"""
    
    def __init__(self):
        super().__init__('hand_eye_calibration')
        
        # 声明参数
        self.declare_parameter('aruco_dict', 'DICT_4X4_50')
        self.declare_parameter('marker_size', 0.05)  # 标记边长 (米)
        self.declare_parameter('marker_id', 0)  # 目标标记ID
        self.declare_parameter('camera_topic', '/camera/camera/color/image_rect_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('base_link', 'base_link')
        self.declare_parameter('end_effector_link', 'tool0')
        self.declare_parameter('min_samples', 15)  # 最小采样数
        self.declare_parameter('calibration_method', 'daniilidis')  # 标定算法
        self.declare_parameter('output_file', 'hand_eye_calibration_result.yaml')
        
        # 获取参数
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
        
        # 初始化 ArUco 检测器
        self._init_aruco_detector()
        
        # 初始化 CV Bridge
        self.bridge = CvBridge()
        
        # 相机内参
        self.camera_matrix = None
        self.dist_coeffs = None
        self.camera_info_received = False
        
        # 当前图像和检测结果
        self.current_image = None
        self.current_rvec = None
        self.current_tvec = None
        self.marker_detected = False
        self.image_lock = threading.Lock()
        
        # ArUco检测去噪：多帧历史缓冲区
        self.detection_history_size = 10
        self.detection_min_frames = 5
        self.detection_rvec_history = []
        self.detection_tvec_history = []
        self.denoised_rvec = None
        self.denoised_tvec = None
        self.detection_stable = False
        
        # 采样数据存储
        self.R_gripper2base_samples = []
        self.t_gripper2base_samples = []
        self.R_target2cam_samples = []
        self.t_target2cam_samples = []
        
        # Alicia-M 机械臂关节名称（小写）
        self.arm_joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        
        # 机械臂控制 Action 客户端
        self.arm_action_client = ActionClient(
            self, 
            FollowJointTrajectory, 
            '/arm_controller/follow_joint_trajectory'
        )
        
        # 夹爪控制 Action 客户端
        self.gripper_action_client = ActionClient(
            self,
            GripperCommand,
            '/gripper_controller/gripper_cmd'
        )
        
        # MoveIt ExecuteTrajectory action 客户端
        self.execute_trajectory_client = ActionClient(
            self, ExecuteTrajectory, '/execute_trajectory')
        
        # 运动规划服务客户端
        self.plan_client = self.create_client(GetMotionPlan, '/plan_kinematic_path')
        
        # TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # QoS 配置
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # 订阅相机话题
        self.image_sub = self.create_subscription(
            Image,
            self.camera_topic,
            self._image_callback,
            qos_profile
        )
        
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self._camera_info_callback,
            10
        )
        
        # 图像显示定时器
        self.display_timer = self.create_timer(0.033, self._display_image)
        
        # 标定位姿列表（针对Alicia-M重新设计）
        self.calibration_poses = self._generate_calibration_poses()
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('Alicia-M 手眼标定节点已启动 (Eye-in-Hand)')
        self.get_logger().info(f'ArUco 字典: {self.aruco_dict_name}')
        self.get_logger().info(f'标记大小: {self.marker_size * 100:.1f} cm')
        self.get_logger().info(f'目标标记ID: {self.marker_id}')
        self.get_logger().info(f'最小采样数: {self.min_samples}')
        self.get_logger().info(f'标定算法: {self.calibration_method}')
        self.get_logger().info(f'末端坐标系: {self.end_effector_link}')
        self.get_logger().info('TF树: tool0 -> camera_link -> camera_color_optical_frame -> aruco_marker')
        self.get_logger().info('=' * 60)
    
    def _init_aruco_detector(self):
        """初始化 ArUco 检测器"""
        aruco_dict_map = {
            'DICT_4X4_50': cv2.aruco.DICT_4X4_50,
            'DICT_4X4_100': cv2.aruco.DICT_4X4_100,
            'DICT_4X4_250': cv2.aruco.DICT_4X4_250,
            'DICT_4X4_1000': cv2.aruco.DICT_4X4_1000,
            'DICT_5X5_50': cv2.aruco.DICT_5X5_50,
            'DICT_5X5_100': cv2.aruco.DICT_5X5_100,
            'DICT_5X5_250': cv2.aruco.DICT_5X5_250,
            'DICT_5X5_1000': cv2.aruco.DICT_5X5_1000,
            'DICT_6X6_50': cv2.aruco.DICT_6X6_50,
            'DICT_6X6_100': cv2.aruco.DICT_6X6_100,
            'DICT_6X6_250': cv2.aruco.DICT_6X6_250,
            'DICT_6X6_1000': cv2.aruco.DICT_6X6_1000,
            'DICT_7X7_50': cv2.aruco.DICT_7X7_50,
            'DICT_7X7_100': cv2.aruco.DICT_7X7_100,
            'DICT_7X7_250': cv2.aruco.DICT_7X7_250,
            'DICT_7X7_1000': cv2.aruco.DICT_7X7_1000,
        }
        
        if self.aruco_dict_name not in aruco_dict_map:
            self.get_logger().warn(f'未知的 ArUco 字典: {self.aruco_dict_name}, 使用默认 DICT_4X4_50')
            self.aruco_dict_name = 'DICT_4X4_50'
        
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_map[self.aruco_dict_name])
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
    
    def _generate_calibration_poses(self):
        """
        生成标定用的机械臂位姿（针对Alicia-M设计）
        
        基准位姿（相机垂直向下正对aruco码）:
        [joint1, joint2, joint3, joint4, joint5, joint6] = [0.0, -1.09, -0.87, 0.0, -0.77, 0.0]
        
        设计原则:
        1. 基于当前正对aruco的位姿进行小范围调整
        2. 确保机械臂末端不会撞到桌面
        3. 提供足够的旋转多样性用于手眼标定
        
        Alicia-M 关节限位:
        - joint1: [-2.75, 2.75] rad
        - joint2: [-3.14, 0] rad
        - joint3: [-3.14, 0] rad
        - joint4: [-1.57, 1.57] rad
        - joint5: [-1.57, 1.57] rad
        - joint6: [-2.75, 2.75] rad
        """
        poses = []
        
        # 基准位姿: [0.0, -1.09, -0.87, 0.0, -0.77, 0.0]
        # 相机垂直向下正对aruco码
        
        # 位姿1: 基准位姿
        poses.append([0.0, -1.09, -0.87, 0.0, -0.77, 0.0])
        
        # 位姿2-4: 通过joint1左右小范围旋转
        poses.append([-0.12, -1.09, -0.87, 0.0, -0.77, 0.0])
        poses.append([0.12, -1.09, -0.87, 0.0, -0.77, 0.0])
        poses.append([-0.20, -1.09, -0.87, 0.0, -0.77, 0.0])
        
        # 位姿5-7: 通过joint6旋转末端（绕相机光轴旋转）
        poses.append([0.0, -1.09, -0.87, 0.0, -0.77, 0.4])
        poses.append([0.0, -1.09, -0.87, 0.0, -0.77, -0.4])
        poses.append([0.0, -1.09, -0.87, 0.0, -0.77, 0.8])
        
        # 位姿8-10: 通过joint5调整手腕倾斜
        poses.append([0.0, -1.09, -0.87, 0.0, -0.65, 0.0])
        poses.append([0.0, -1.09, -0.87, 0.0, -0.90, 0.0])
        poses.append([0.0, -1.09, -0.87, 0.0, -0.55, 0.0])
        
        # 位姿11-13: 通过joint4调整
        poses.append([0.0, -1.09, -0.87, 0.25, -0.77, 0.0])
        poses.append([0.0, -1.09, -0.87, -0.25, -0.77, 0.0])
        poses.append([0.0, -1.09, -0.87, 0.4, -0.77, 0.0])
        
        # 位姿14-16: joint1 + joint6 组合
        poses.append([0.15, -1.09, -0.87, 0.0, -0.77, 0.5])
        poses.append([-0.15, -1.09, -0.87, 0.0, -0.77, -0.5])
        poses.append([0.10, -1.09, -0.87, 0.0, -0.77, -0.6])
        
        # 位姿17-19: joint4 + joint5 + joint6 组合
        poses.append([0.0, -1.09, -0.87, 0.2, -0.70, 0.3])
        poses.append([0.0, -1.09, -0.87, -0.2, -0.85, -0.3])
        poses.append([0.0, -1.09, -0.87, 0.15, -0.60, 0.5])
        
        # 位姿20: 多轴组合
        poses.append([-0.10, -1.09, -0.87, -0.15, -0.70, -0.4])
        
        return poses
    
    def _image_callback(self, msg):
        """图像回调函数"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            with self.image_lock:
                self.current_image = cv_image.copy()
                
                if self.camera_matrix is not None:
                    self._detect_aruco(cv_image)
                    
        except Exception as e:
            self.get_logger().error(f'图像处理错误: {e}')
    
    def _camera_info_callback(self, msg):
        """相机信息回调函数"""
        if not self.camera_info_received:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.camera_info_received = True
            self.get_logger().info('相机内参已接收')
            self.get_logger().info(f'相机矩阵:\n{self.camera_matrix}')
    
    def _detect_aruco(self, image):
        """检测 ArUco 标记并估计位姿"""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        corners, ids, rejected = self.aruco_detector.detectMarkers(gray)
        
        self.marker_detected = False
        self.current_rvec = None
        self.current_tvec = None
        
        if ids is not None and len(ids) > 0:
            for i, marker_id in enumerate(ids.flatten()):
                if marker_id == self.marker_id:
                    obj_points = np.array([
                        [-self.marker_size/2, self.marker_size/2, 0],
                        [self.marker_size/2, self.marker_size/2, 0],
                        [self.marker_size/2, -self.marker_size/2, 0],
                        [-self.marker_size/2, -self.marker_size/2, 0]
                    ], dtype=np.float32)
                    
                    success, rvec, tvec = cv2.solvePnP(
                        obj_points, corners[i][0], self.camera_matrix, self.dist_coeffs
                    )
                    
                    if success:
                        self.current_rvec = rvec.flatten()
                        self.current_tvec = tvec.flatten()
                        self.marker_detected = True
                        self._add_detection_to_history(self.current_rvec, self.current_tvec)
                    break
        
        if not self.marker_detected:
            self._add_detection_to_history(None, None)
    
    def _add_detection_to_history(self, rvec, tvec):
        """添加检测结果到历史缓冲区并计算去噪结果"""
        self.detection_rvec_history.append(rvec)
        self.detection_tvec_history.append(tvec)
        
        if len(self.detection_rvec_history) > self.detection_history_size:
            self.detection_rvec_history.pop(0)
            self.detection_tvec_history.pop(0)
        
        self._compute_denoised_pose()
    
    def _compute_denoised_pose(self):
        """从历史数据计算去噪后的位姿"""
        valid_rvecs = [r for r in self.detection_rvec_history if r is not None]
        valid_tvecs = [t for t in self.detection_tvec_history if t is not None]
        
        if len(valid_rvecs) < self.detection_min_frames:
            self.detection_stable = False
            self.denoised_rvec = None
            self.denoised_tvec = None
            return
        
        rvec_array = np.array(valid_rvecs)
        tvec_array = np.array(valid_tvecs)
        
        tvec_std = np.std(tvec_array, axis=0)
        max_std = np.max(tvec_std)
        
        if max_std > 0.005:
            self.detection_stable = False
            self.denoised_rvec = None
            self.denoised_tvec = None
            return
        
        self.denoised_rvec = np.median(rvec_array, axis=0)
        self.denoised_tvec = np.median(tvec_array, axis=0)
        self.detection_stable = True
    
    def clear_detection_history(self):
        """清空检测历史"""
        self.detection_rvec_history = []
        self.detection_tvec_history = []
        self.denoised_rvec = None
        self.denoised_tvec = None
        self.detection_stable = False
    
    def _display_image(self):
        """显示图像窗口"""
        with self.image_lock:
            if self.current_image is None:
                return
            
            display_img = self.current_image.copy()
        
        if self.camera_matrix is not None:
            gray = cv2.cvtColor(display_img, cv2.COLOR_BGR2GRAY)
            corners, ids, rejected = self.aruco_detector.detectMarkers(gray)
            
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(display_img, corners, ids)
                
                for i, marker_id in enumerate(ids.flatten()):
                    if marker_id == self.marker_id:
                        obj_points = np.array([
                            [-self.marker_size/2, self.marker_size/2, 0],
                            [self.marker_size/2, self.marker_size/2, 0],
                            [self.marker_size/2, -self.marker_size/2, 0],
                            [-self.marker_size/2, -self.marker_size/2, 0]
                        ], dtype=np.float32)
                        
                        success, rvec, tvec = cv2.solvePnP(
                            obj_points, corners[i][0], self.camera_matrix, self.dist_coeffs
                        )
                        
                        if success:
                            cv2.drawFrameAxes(display_img, self.camera_matrix, self.dist_coeffs,
                                             rvec, tvec, self.marker_size * 0.5)
                            
                            distance = np.linalg.norm(tvec)
                            cv2.putText(display_img, f'Distance: {distance:.3f}m', 
                                       (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        status_color = (0, 255, 0) if self.marker_detected else (0, 0, 255)
        status_text = f'Marker {self.marker_id}: {"Detected" if self.marker_detected else "Not Found"}'
        cv2.putText(display_img, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
        
        samples_text = f'Samples: {len(self.R_gripper2base_samples)}/{self.min_samples}'
        cv2.putText(display_img, samples_text, (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        
        stable_color = (0, 255, 0) if self.detection_stable else (0, 165, 255)
        stable_text = f'Stable: {"Yes" if self.detection_stable else "No"} ({len([r for r in self.detection_rvec_history if r is not None])}/{self.detection_history_size})'
        cv2.putText(display_img, stable_text, (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, stable_color, 2)
        
        cv2.imshow('Alicia-M Hand-Eye Calibration - ArUco Detection', display_img)
        cv2.waitKey(1)
    
    def open_gripper(self):
        """张开夹爪"""
        self.get_logger().info('张开夹爪...')
        
        if not self.gripper_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn('夹爪Action服务器不可用，跳过夹爪控制')
            return True
        
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = 0.0  # 0.0 = 全开
        goal_msg.command.max_effort = 10.0
        
        send_goal_future = self.gripper_action_client.send_goal_async(goal_msg)
        
        timeout_sec = 5.0
        start_time = time.time()
        while not send_goal_future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.time() - start_time > timeout_sec:
                self.get_logger().warn('等待夹爪目标接受超时')
                return False
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('夹爪目标被拒绝')
            return False
        
        result_future = goal_handle.get_result_async()
        
        timeout_sec = 5.0
        start_time = time.time()
        while not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.time() - start_time > timeout_sec:
                self.get_logger().warn('等待夹爪结果超时')
                return False
        
        self.get_logger().info('夹爪已张开')
        return True
    
    def wait_for_services(self):
        """等待所有必要的服务"""
        self.get_logger().info('等待机械臂 Action 服务器...')
        if not self.arm_action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('无法连接到机械臂 Action 服务器')
            return False
        self.get_logger().info('机械臂 Action 服务器已连接')
        
        self.get_logger().info('等待ExecuteTrajectory服务器...')
        if not self.execute_trajectory_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('无法连接到ExecuteTrajectory服务器')
            return False
        self.get_logger().info('ExecuteTrajectory服务器已连接')
        
        self.get_logger().info('等待运动规划服务...')
        if not self.plan_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('无法连接到运动规划服务')
            return False
        self.get_logger().info('运动规划服务已连接')
        
        self.get_logger().info('等待相机数据...')
        timeout = 30.0
        start_time = time.time()
        while not self.camera_info_received:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start_time > timeout:
                self.get_logger().error('等待相机数据超时')
                return False
        
        self.get_logger().info('相机数据已就绪')
        return True
    
    def plan_to_joint_positions(self, joint_positions):
        """
        使用MoveIt规划到目标关节位置的轨迹
        """
        request = GetMotionPlan.Request()
        
        # Alicia-M 使用 "arm" 规划组
        request.motion_plan_request.group_name = "arm"
        request.motion_plan_request.num_planning_attempts = 5
        request.motion_plan_request.allowed_planning_time = 5.0
        request.motion_plan_request.max_velocity_scaling_factor = 0.3
        request.motion_plan_request.max_acceleration_scaling_factor = 0.3
        
        request.motion_plan_request.start_state.is_diff = True
        
        goal_constraints = Constraints()
        goal_constraints.name = "joint_goal"
        
        for i, joint_name in enumerate(self.arm_joint_names):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = joint_name
            joint_constraint.position = float(joint_positions[i])
            joint_constraint.tolerance_above = 0.01
            joint_constraint.tolerance_below = 0.01
            joint_constraint.weight = 1.0
            goal_constraints.joint_constraints.append(joint_constraint)
        
        request.motion_plan_request.goal_constraints.append(goal_constraints)
        
        try:
            self.get_logger().info(f'规划到关节位置: {[f"{j:.3f}" for j in joint_positions]}')
            future = self.plan_client.call_async(request)
            
            timeout_sec = 10.0
            start_time = time.time()
            while not future.done():
                rclpy.spin_once(self, timeout_sec=0.05)
                if time.time() - start_time > timeout_sec:
                    self.get_logger().error('运动规划服务超时')
                    return None
            
            response = future.result()
            
            if response.motion_plan_response.error_code.val == 1:
                self.get_logger().info(f'规划成功，轨迹包含 {len(response.motion_plan_response.trajectory.joint_trajectory.points)} 个点')
                return response.motion_plan_response.trajectory
            else:
                self.get_logger().error(f'运动规划失败，错误码: {response.motion_plan_response.error_code.val}')
                return None
                
        except Exception as e:
            self.get_logger().error(f'运动规划服务调用失败: {e}')
            return None
    
    def execute_trajectory(self, trajectory):
        """
        执行规划好的轨迹
        """
        goal_msg = ExecuteTrajectory.Goal()
        goal_msg.trajectory = trajectory
        
        self.get_logger().info('执行轨迹...')
        
        send_goal_future = self.execute_trajectory_client.send_goal_async(goal_msg)
        
        timeout_sec = 10.0
        start_time = time.time()
        while not send_goal_future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.time() - start_time > timeout_sec:
                self.get_logger().error('等待轨迹执行目标接受超时')
                return False
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('轨迹执行目标被拒绝')
            return False
        
        self.get_logger().info('轨迹执行目标已接受，等待完成...')
        
        if trajectory.joint_trajectory.points:
            last_point = trajectory.joint_trajectory.points[-1]
            traj_duration = last_point.time_from_start.sec + last_point.time_from_start.nanosec / 1e9
            timeout_sec = traj_duration + 15.0
        else:
            timeout_sec = 30.0
        
        result_future = goal_handle.get_result_async()
        
        start_time = time.time()
        while not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.time() - start_time > timeout_sec:
                self.get_logger().error('等待轨迹执行结果超时')
                return False
        
        result = result_future.result()
        if result.result.error_code.val == 1:
            self.get_logger().info('轨迹执行完成')
            return True
        else:
            self.get_logger().error(f'轨迹执行失败，错误码: {result.result.error_code.val}')
            return False
    
    def move_to_pose(self, joint_positions, duration_sec=4.0):
        """
        移动机械臂到指定关节位置
        """
        self.get_logger().info(f'移动到位姿: {[f"{p:.3f}" for p in joint_positions]} (使用MoveIt)')
        
        trajectory = self.plan_to_joint_positions(joint_positions)
        if trajectory is not None:
            if self.execute_trajectory(trajectory):
                time.sleep(0.5)
                return True
        
        self.get_logger().warn('MoveIt plan+execute失败，尝试直接轨迹控制...')
        return self._move_to_pose_direct(joint_positions, duration_sec)
    
    def _move_to_pose_direct(self, joint_positions, duration_sec=4.0):
        """
        直接使用FollowJointTrajectory控制机械臂（备用方法）
        """
        goal_msg = FollowJointTrajectory.Goal()
        
        trajectory = JointTrajectory()
        trajectory.joint_names = self.arm_joint_names
        
        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start = Duration(seconds=duration_sec).to_msg()
        
        trajectory.points.append(point)
        goal_msg.trajectory = trajectory
        
        self.get_logger().info(f'直接控制移动到位姿: {[f"{p:.3f}" for p in joint_positions]}')
        
        send_goal_future = self.arm_action_client.send_goal_async(goal_msg)
        
        timeout_sec = 10.0
        start_time = time.time()
        while not send_goal_future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.time() - start_time > timeout_sec:
                self.get_logger().error('等待目标接受超时')
                return False
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('目标被拒绝')
            return False
        
        result_future = goal_handle.get_result_async()
        
        timeout_sec = duration_sec + 10.0
        start_time = time.time()
        while not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.time() - start_time > timeout_sec:
                self.get_logger().error('等待运动结果超时')
                return False
        
        result = result_future.result()
        if result.result.error_code == 0:
            self.get_logger().info('到达目标位置')
            return True
        else:
            self.get_logger().error(f'运动失败，错误码: {result.result.error_code}')
            return False
    
    def get_end_effector_pose(self):
        """
        获取末端执行器相对于基座的位姿
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_link,
                self.end_effector_link,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0)
            )
            
            q = transform.transform.rotation
            t = transform.transform.translation
            
            rotation = R.from_quat([q.x, q.y, q.z, q.w])
            rotation_matrix = rotation.as_matrix()
            
            translation_vector = np.array([t.x, t.y, t.z])
            
            return rotation_matrix, translation_vector
            
        except Exception as e:
            self.get_logger().error(f'获取末端位姿失败: {e}')
            return None
    
    def get_marker_pose(self):
        """
        获取 ArUco 标记相对于相机的位姿
        """
        if self.detection_stable and self.denoised_rvec is not None:
            rotation_matrix, _ = cv2.Rodrigues(self.denoised_rvec)
            translation_vector = self.denoised_tvec
            return rotation_matrix, translation_vector
        
        if not self.marker_detected or self.current_rvec is None:
            return None
        
        rotation_matrix, _ = cv2.Rodrigues(self.current_rvec)
        translation_vector = self.current_tvec
        
        return rotation_matrix, translation_vector
    
    def record_sample(self):
        """
        记录当前采样点
        """
        ee_pose = self.get_end_effector_pose()
        if ee_pose is None:
            self.get_logger().warn('无法获取末端位姿')
            return False
        
        marker_pose = self.get_marker_pose()
        if marker_pose is None:
            self.get_logger().warn('无法获取标记位姿 (标记未检测到)')
            return False
        
        R_gripper2base, t_gripper2base = ee_pose
        R_target2cam, t_target2cam = marker_pose
        
        self.R_gripper2base_samples.append(R_gripper2base)
        self.t_gripper2base_samples.append(t_gripper2base)
        self.R_target2cam_samples.append(R_target2cam)
        self.t_target2cam_samples.append(t_target2cam)
        
        sample_count = len(self.R_gripper2base_samples)
        self.get_logger().info(f'成功记录采样点 #{sample_count}')
        self.get_logger().info(f'  末端位置: [{t_gripper2base[0]:.4f}, {t_gripper2base[1]:.4f}, {t_gripper2base[2]:.4f}]')
        self.get_logger().info(f'  标记距离: {np.linalg.norm(t_target2cam):.4f} m')
        
        return True
    
    def compute_calibration(self):
        """
        计算手眼标定结果
        """
        if len(self.R_gripper2base_samples) < 3:
            self.get_logger().error('采样数量不足，至少需要3个采样点')
            return None
        
        self.get_logger().info(f'开始计算手眼标定 (共 {len(self.R_gripper2base_samples)} 个采样点)')
        
        method_map = {
            'tsai': cv2.CALIB_HAND_EYE_TSAI,
            'park': cv2.CALIB_HAND_EYE_PARK,
            'horaud': cv2.CALIB_HAND_EYE_HORAUD,
            'daniilidis': cv2.CALIB_HAND_EYE_DANIILIDIS,
            'andreff': cv2.CALIB_HAND_EYE_ANDREFF
        }
        
        if self.calibration_method not in method_map:
            self.get_logger().warn(f'未知的标定算法: {self.calibration_method}, 使用 tsai')
            self.calibration_method = 'tsai'
        
        method = method_map[self.calibration_method]
        
        R_gripper2base = [r for r in self.R_gripper2base_samples]
        t_gripper2base = [t.reshape(3, 1) for t in self.t_gripper2base_samples]
        R_target2cam = [r for r in self.R_target2cam_samples]
        t_target2cam = [t.reshape(3, 1) for t in self.t_target2cam_samples]
        
        try:
            R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
                R_gripper2base, t_gripper2base,
                R_target2cam, t_target2cam,
                method=method
            )
            
            self.get_logger().info('手眼标定计算完成')
            return R_cam2gripper, t_cam2gripper.flatten()
            
        except Exception as e:
            self.get_logger().error(f'手眼标定计算失败: {e}')
            return None
    
    def compute_with_multiple_methods(self):
        """
        使用多种算法计算并比较结果
        """
        methods = {
            'tsai': cv2.CALIB_HAND_EYE_TSAI,
            'park': cv2.CALIB_HAND_EYE_PARK,
            'horaud': cv2.CALIB_HAND_EYE_HORAUD,
            'daniilidis': cv2.CALIB_HAND_EYE_DANIILIDIS,
            'andreff': cv2.CALIB_HAND_EYE_ANDREFF
        }
        
        results = {}
        
        R_gripper2base = [r for r in self.R_gripper2base_samples]
        t_gripper2base = [t.reshape(3, 1) for t in self.t_gripper2base_samples]
        R_target2cam = [r for r in self.R_target2cam_samples]
        t_target2cam = [t.reshape(3, 1) for t in self.t_target2cam_samples]
        
        for name, method in methods.items():
            try:
                R, t = cv2.calibrateHandEye(
                    R_gripper2base, t_gripper2base,
                    R_target2cam, t_target2cam,
                    method=method
                )
                results[name] = {
                    'R': R,
                    't': t.flatten(),
                    'success': True
                }
            except Exception as e:
                results[name] = {
                    'error': str(e),
                    'success': False
                }
        
        return results
    
    def save_result(self, R_cam2gripper, t_cam2gripper):
        """
        保存标定结果到 YAML 文件
        """
        rotation = R.from_matrix(R_cam2gripper)
        quaternion = rotation.as_quat()
        euler = rotation.as_euler('xyz', degrees=True)
        
        transform_matrix = np.eye(4)
        transform_matrix[:3, :3] = R_cam2gripper
        transform_matrix[:3, 3] = t_cam2gripper
        
        result = {
            'hand_eye_calibration': {
                'type': 'eye_in_hand',
                'frame_id': self.end_effector_link,
                'child_frame_id': 'camera_link',
                'date': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                'samples_count': len(self.R_gripper2base_samples),
                'calibration_method': self.calibration_method,
                'aruco_marker_id': self.marker_id,
                'aruco_marker_size': self.marker_size,
                'transform': {
                    'translation': {
                        'x': float(t_cam2gripper[0]),
                        'y': float(t_cam2gripper[1]),
                        'z': float(t_cam2gripper[2])
                    },
                    'rotation': {
                        'quaternion': {
                            'x': float(quaternion[0]),
                            'y': float(quaternion[1]),
                            'z': float(quaternion[2]),
                            'w': float(quaternion[3])
                        },
                        'euler_xyz_deg': {
                            'roll': float(euler[0]),
                            'pitch': float(euler[1]),
                            'yaw': float(euler[2])
                        }
                    },
                    'matrix_4x4': transform_matrix.tolist()
                }
            }
        }
        
        try:
            package_share_dir = get_package_share_directory('alicia_m_calibration')
            workspace_root = os.path.abspath(os.path.join(package_share_dir, '..', '..', '..', '..'))
            config_dir = os.path.join(workspace_root, 'src', 'alicia_m_calibration', 'config')
        except Exception as e:
            self.get_logger().warn(f'无法获取包路径: {e}，使用脚本相对路径')
            script_dir = os.path.dirname(__file__)
            config_dir = os.path.join(script_dir, '..', 'config')
        
        output_path = os.path.join(config_dir, self.output_file)
        
        os.makedirs(config_dir, exist_ok=True)
        
        with open(output_path, 'w') as f:
            yaml.dump(result, f, default_flow_style=False, allow_unicode=True)
        
        abs_path = os.path.abspath(output_path)
        self.get_logger().info(f'标定结果已保存到: {abs_path}')
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('手眼标定结果 (相机到末端执行器 tool0)')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'平移 (m): x={t_cam2gripper[0]:.6f}, y={t_cam2gripper[1]:.6f}, z={t_cam2gripper[2]:.6f}')
        self.get_logger().info(f'旋转 (四元数): x={quaternion[0]:.6f}, y={quaternion[1]:.6f}, z={quaternion[2]:.6f}, w={quaternion[3]:.6f}')
        self.get_logger().info(f'旋转 (欧拉角, 度): roll={euler[0]:.2f}, pitch={euler[1]:.2f}, yaw={euler[2]:.2f}')
        self.get_logger().info('=' * 60)
        
        return output_path
    
    def run_calibration(self):
        """运行标定流程"""
        self.get_logger().info('=' * 60)
        self.get_logger().info('开始 Alicia-M 手眼标定流程')
        self.get_logger().info('=' * 60)
        
        if not self.wait_for_services():
            return False
        
        # 首先张开夹爪
        self.get_logger().info('准备标定：张开夹爪...')
        self.open_gripper()
        time.sleep(1.0)
        
        # 移动到第一个标定位姿
        first_pose = self.calibration_poses[0]
        self.get_logger().info('移动到初始标定位置...')
        if not self.move_to_pose(first_pose, 4.0):
            self.get_logger().error('无法移动到初始标定位置')
            return False
        
        time.sleep(1.0)
        
        # 遍历标定位姿
        pose_idx = 0
        while len(self.R_gripper2base_samples) < self.min_samples and pose_idx < len(self.calibration_poses):
            self.get_logger().info(f'\n--- 位姿 {pose_idx + 1}/{len(self.calibration_poses)} ---')
            
            pose = self.calibration_poses[pose_idx]
            
            if not self.move_to_pose(pose, 4.0):
                self.get_logger().warn(f'位姿 {pose_idx + 1} 移动失败，跳过')
                pose_idx += 1
                continue
            
            self.get_logger().info('等待机械臂稳定 ...')
            settle_start = time.time()
            while time.time() - settle_start < 2.0:
                rclpy.spin_once(self, timeout_sec=0.05)
            
            self.clear_detection_history()
            
            self.get_logger().info('收集检测数据...')
            stable_wait_start = time.time()
            max_stable_wait = 5.0
            
            while time.time() - stable_wait_start < max_stable_wait:
                rclpy.spin_once(self, timeout_sec=0.05)
                
                if self.detection_stable:
                    self.get_logger().info('检测已稳定')
                    break
            
            if not self.detection_stable:
                self.get_logger().warn(f'位姿 {pose_idx + 1}: 标记检测不稳定，跳过')
                pose_idx += 1
                continue
            
            self.get_logger().info(f'位姿 {pose_idx + 1}: 标记检测稳定，自动记录采样点...')
            if self.record_sample():
                self.get_logger().info(f'已记录采样点 {len(self.R_gripper2base_samples)}/{self.min_samples}')
            else:
                self.get_logger().warn(f'采样点记录失败，跳过')
            
            pose_idx += 1
        
        # 回到第一个位姿
        self.get_logger().info('返回初始位置...')
        self.move_to_pose(self.calibration_poses[0], 3.0)
        
        if len(self.R_gripper2base_samples) < 3:
            self.get_logger().error(f'采样数量不足: {len(self.R_gripper2base_samples)} (最少需要3个)')
            return False
        
        if len(self.R_gripper2base_samples) < self.min_samples:
            self.get_logger().warn(f'采样数量低于推荐值: {len(self.R_gripper2base_samples)}/{self.min_samples}，但继续计算')
        
        self.get_logger().info('\n使用多种算法计算手眼标定...')
        all_results = self.compute_with_multiple_methods()
        
        self.get_logger().info('\n各算法计算结果比较:')
        for name, result in all_results.items():
            if result['success']:
                t = result['t']
                self.get_logger().info(f'  {name}: t=[{t[0]:.4f}, {t[1]:.4f}, {t[2]:.4f}]')
            else:
                self.get_logger().warn(f'  {name}: 计算失败 - {result.get("error", "未知错误")}')
        
        if self.calibration_method in all_results and all_results[self.calibration_method]['success']:
            R_result = all_results[self.calibration_method]['R']
            t_result = all_results[self.calibration_method]['t']
        else:
            for name, result in all_results.items():
                if result['success']:
                    R_result = result['R']
                    t_result = result['t']
                    self.calibration_method = name
                    break
            else:
                self.get_logger().error('所有算法都计算失败')
                return False
        
        self.save_result(R_result, t_result)
        
        self.get_logger().info('手眼标定完成!')
        return True
    
    def shutdown(self):
        """关闭节点"""
        cv2.destroyAllWindows()


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    node = HandEyeCalibration()
    
    try:
        success = node.run_calibration()
        if success:
            print('\n标定完成! 请查看保存的标定结果文件。')
        else:
            print('\n标定失败或被用户取消。')
    except KeyboardInterrupt:
        print('\n用户中断')
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()
    
    return 0 if success else 1


if __name__ == '__main__':
    sys.exit(main())
