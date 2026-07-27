# spray_task.py
"""
ROS 任务节点：Alicia-M 机械臂果园喷洒作业工作流 (Orchard Spraying Workflow)。

本节点是一个长时 Action Server，负责执行 `/arm/execute_spray` 接口。
它集成了视觉目标、IK/预设观察、下游 Action 三类职责；复杂观察策略分别位于
`ik_observation.py`、`joint_preset_observation.py` 与 `observation_flow.py`。

核心业务流程：
MOVING_TO_OBSERVE (观察位姿) -> DETECTING_TARGETS (病态目标检测)
-> QUEUING (去重排队) -> ALIGNING (重心+视觉伺服对准) -> SPRAYING (喷洒)
-> RETURNING_TO_OBSERVE (返回观察位, 复检) -> RETURNING_HOME (任务结束归位)
"""

import math
import threading

from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data)
from sensor_msgs.msg import CameraInfo, JointState
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener
from vision_msgs.msg import Detection2DArray
from wvcsc_interfaces.action import AlignTarget, ExecuteSpray, Spray
from wvcsc_interfaces.msg import Target2D
from wvcsc_interfaces.srv import ComputeSprayAim

from .action_flow import DownstreamActionMixin
from .ik_observation import ObservationOptimizer
from .joint_preset_observation import JointPresetObservationMixin
from .motion_state import MotionControlState
from .node_parameters import create_alicia_moveit
from .observation_flow import ObservationFlowMixin
from .spray_aim import (
    SprayAimMixin,
    WORKING_RANGE_MAX_M,
    WORKING_RANGE_MIN_M,
)
from .spray_config import (
    SprayConfig,
    declare_spray_parameters,
)
from .spray_sequence import SpraySequenceMixin
from .target_flow import TargetFlowMixin


_OBSERVATION_MODES = {'ik', 'joint_presets'}


class SprayTask(
        TargetFlowMixin, JointPresetObservationMixin, ObservationFlowMixin,
        DownstreamActionMixin, SprayAimMixin, SpraySequenceMixin, Node):
    """
    协调 MoveIt、YOLO、视觉伺服和喷洒执行器的长时 Action Server。

    订阅回调只更新受互斥锁保护的最新视觉/关节快照，Action 执行线程运行状态机；
    ``MotionControlState`` 的锁定和 cancel epoch 始终优先于任务推进。失败恢复只会
    尝试已筛选的下一观察位，不能通过降低碰撞、奇异点或关节余量阈值强行执行。
    """

    def __init__(self):
        super().__init__('wvcsc_spray_task')
        self._declare_parameters()
        self.config = SprayConfig.from_node(self)

        # === 1. 运动学与动作基础配置 ===
        self._home = self.config.home
        self._min_duration = self.config.min_duration
        self._max_duration = self.config.max_duration
        self._vision_timeout = self.config.vision_timeout
        self._downstream_server_timeout = self.config.downstream_server_timeout
        self._downstream_margin = self.config.downstream_margin
        self._camera_frame = self.config.camera_frame
        self._base_frame = self.config.base_frame
        self._spray_on_alignment_failure = (
            self.config.spray_on_alignment_failure)
        self._observation_mode = self.config.observation_mode
        self._joint_preset_positions = self.config.joint_preset_positions
        self._joint_preset_side_epsilon_m = (
            self.config.joint_preset_side_epsilon_m)
        self._joint_preset_side = ''
        self._observation_config = dict(self.config.observation)
        self._recenter_config = dict(self.config.recenter)

        # 核心组件 1：观察优化器 (基于 URDF 和实时 IK 筛选安全观察位)
        self._observation_optimizer = ObservationOptimizer(
            self.config.robot_description,
            self._base_frame,
            'tool0',
            self.arm_joint_names,
            self._observation_config)

        # 核心组件 2：状态机安全锁与运动适配器
        self.state = MotionControlState()
        self.arm, self._callback_group = create_alicia_moveit(self, self.state)

        # 核心组件 3：TF 变换缓冲
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # 核心组件 4：下游动作客户端 (视觉对齐和喷洒)
        self._vision_client = ActionClient(
            self, AlignTarget, self.config.vision_action_name,
            callback_group=self._callback_group)
        self._aim_client = self.create_client(
            ComputeSprayAim,
            self.config.aim_service_name,
            callback_group=self._callback_group)
        self._spray_client = ActionClient(
            self, Spray, self.config.spray_action_name,
            callback_group=self._callback_group)

        # 核心组件 5：话题发布与订阅
        self._selected_target_pub = self.create_publisher(
            String, self.config.selected_target_topic, 10)
        self._motion_command_pub = self.create_publisher(
            String, '/motion_control/command', 10)

        # 推理模式切换（病态目标发现与单目标跟踪）
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._inference_mode_pub = self.create_publisher(
            String, self.config.inference_mode_topic, latched)

        # 视觉与传感器订阅（数据由 `TargetFlowMixin` 和 `ObservationFlowMixin` 处理）
        self.create_subscription(
            Bool, self.config.motion_locked_topic,
            self._on_motion_locked, latched, callback_group=self._callback_group)
        self.create_subscription(
            Detection2DArray, self.config.diseased_target_detection_topic,
            self._on_fruit_detections, 10, callback_group=self._callback_group)
        self.create_subscription(
            Target2D, self.config.vision_target_topic,
            self._on_selected_target, 10, callback_group=self._callback_group)
        self.create_subscription(
            CameraInfo, self.config.camera_info_topic,
            self._on_camera_info, qos_profile_sensor_data,
            callback_group=self._callback_group)
        self.create_subscription(
            JointState, self.config.joint_state_topic,
            self._on_joint_state, qos_profile_sensor_data,
            callback_group=self._callback_group)

        # 核心组件 6：Action Server (由 `_execute_action` 驱动状态机)
        self._action_server = ActionServer(
            self, ExecuteSpray, '/arm/execute_spray',
            execute_callback=self._execute_action,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._callback_group)

        # === 线程安全的状态变量 ===
        self._abort = threading.Event()          # 取消/急停标志位
        self._busy_mutex = threading.Lock()      # 保证同一时间只有一个 ExecuteSpray 在执行
        self._busy = False
        self._vision_mutex = threading.Lock()    # 保护 YOLO 检测结果的互斥锁
        self._state_mutex = threading.Lock()     # 保护关节/相机模型快照的互斥锁
        self._fruit_frames = 0
        self._fruit_history = []
        self._target_confirmation_id = ''
        self._target_valid_frames = 0
        self._target_confirmation_frames = 0
        self._target_workspace_stable_since = None
        self._target_workspace_last_seen = None
        self._target_workspace_anchor = None
        self._target_workspace_currently_valid = False
        self._latest_selected_target = None
        self._observation_pose = None
        self._observation_candidates = []
        self._observation_candidate_index = -1
        self._observation_distance = None
        self._observation_failure_reason = ''
        self._tree_in_base = None
        self._camera_mount = None
        self._camera_model = None
        self._joint_positions = None
        self._joint_state_sequence = 0
        self._active_mission = ''
        self._active_tree = ''
        self._active_aim = None
        self._working_range_override = 0.0

    @property
    def arm_joint_names(self):
        # Alicia-M 专属六轴关节名称
        return ('joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6')

    def _declare_parameters(self):
        declare_spray_parameters(self)

    # ---------- 传感器与回调 ----------
    def _on_motion_locked(self, message):
        """紧急锁定回调：由 `motion_control` 触发，强制阻断当前所有运动"""
        if message.data:
            self.state.stop()
            self._abort.set()
            self.arm.cancel()
        else:
            self.state.resume()
            if not self._is_busy():
                self._abort.clear()

    def _on_camera_info(self, message):
        """更新相机内参矩阵（用于像素坐标计算）"""
        if message.width <= 0 or message.height <= 0:
            return
        fx, fy, cx, cy = (
            float(message.k[0]), float(message.k[4]),
            float(message.k[2]), float(message.k[5]))
        if min(fx, fy) <= 0.0:
            return
        with self._state_mutex:
            self._camera_model = (fx, fy, cx, cy, int(message.width), int(message.height))

    def _on_joint_state(self, message):
        """更新机械臂当前实际关节角，用于执行 IK 计算"""
        values = dict(zip(message.name, message.position))
        try:
            joints = tuple(float(values[name]) for name in self.arm_joint_names)
        except KeyError:
            return
        if not all(math.isfinite(value) for value in joints):
            return
        with self._state_mutex:
            self._joint_positions = joints
            self._joint_state_sequence += 1

    # ---------- Action Server 生命周期回调 ----------
    def _goal_callback(self, request):
        error = self._validate_goal(request)
        if error or not self._claim():
            self.get_logger().warn(f'[ARM] rejected goal: {error or "busy or locked"}')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle):
        self._abort.set()
        self.arm.cancel()
        self._request_motion_stop()
        return CancelResponse.ACCEPT

    def _execute_action(self, goal_handle):
        """Action Server 主执行线程"""
        request = goal_handle.request
        result = ExecuteSpray.Result()
        previous_observation_mode = self._observation_mode
        previous_working_range = self._working_range_override
        self._observation_mode = self._goal_observation_mode(request)
        self._working_range_override = self._goal_working_range(request)
        try:
            code, message = self._run_sequence(
                request,
                cancel_requested=lambda: goal_handle.is_cancel_requested,
                feedback=lambda phase, progress, text: self._feedback(
                    goal_handle, phase, progress, text))
            result.success = code in {
                ExecuteSpray.Result.OK,
                ExecuteSpray.Result.INSPECTED_NO_DISEASE,
                ExecuteSpray.Result.PARTIAL_SUCCESS,
            }
            result.error_code = code
            result.message = message
            if result.success:
                goal_handle.succeed()
            elif code == ExecuteSpray.Result.CANCELED and goal_handle.is_cancel_requested:
                goal_handle.canceled()
            else:
                goal_handle.abort()
            return result
        except Exception as error:
            self.get_logger().error(f'[ARM] internal error: {error}')
            result.error_code = ExecuteSpray.Result.INTERNAL_ERROR
            result.message = str(error)
            goal_handle.abort()
            return result
        finally:
            self._observation_mode = previous_observation_mode
            self._working_range_override = previous_working_range
            self._select_target('')
            self._set_inference_mode('idle')
            self._active_mission = ''
            self._active_tree = ''
            self._release()

    # ---------- ROS 状态与发布工具 ----------
    def _select_target(self, target_id):
        message = String()
        message.data = target_id
        self._selected_target_pub.publish(message)

    def _set_inference_mode(self, mode):
        message = String()
        message.data = mode
        self._inference_mode_pub.publish(message)

    def _request_motion_stop(self):
        message = String()
        message.data = 'stop'
        self._motion_command_pub.publish(message)

    def _aborted(self, cancel_requested):
        return self._abort.is_set() or cancel_requested()

    @staticmethod
    def _feedback(goal_handle, phase, progress, text):
        message = ExecuteSpray.Feedback()
        message.phase = phase
        message.progress = progress
        message.phase_text = text
        goal_handle.publish_feedback(message)

    def _validate_goal(self, request):
        if not str(request.mission_id).strip() or not str(request.tree_id).strip():
            return 'mission_id and tree_id are required'
        if (not math.isfinite(float(request.spray_duration)) or
                not self._min_duration <= request.spray_duration <= self._max_duration):
            return 'spray_duration out of range'
        if not self._hint_available(request.tree_hint):
            return 'tree_hint in a named frame is required'
        requested_mode = str(
            getattr(request, 'observation_mode', '')).strip().lower()
        if requested_mode and requested_mode not in _OBSERVATION_MODES:
            return 'observation_mode must be ik or joint_presets'
        working_range = self._goal_working_range(request)
        if (not math.isfinite(working_range) or working_range < 0.0 or
                (working_range > 0.0 and not
                 WORKING_RANGE_MIN_M <= working_range <=
                 WORKING_RANGE_MAX_M)):
            return (
                f'working_range_m must be 0 or within '
                f'{WORKING_RANGE_MIN_M:.1f}-{WORKING_RANGE_MAX_M:.1f} m')
        return ''

    def _goal_observation_mode(self, request):
        requested_mode = str(
            getattr(request, 'observation_mode', '')).strip().lower()
        return requested_mode or self.config.observation_mode

    @staticmethod
    def _goal_working_range(request):
        return float(getattr(request, 'working_range_m', 0.0))

    def _claim(self):
        with self._busy_mutex:
            if self._busy or self.state.locked:
                return False
            self._busy = True
            self._abort.clear()
            return True

    def _release(self):
        with self._busy_mutex:
            self._busy = False
            if not self.state.locked:
                self._abort.clear()

    def _is_busy(self):
        with self._busy_mutex:
            return self._busy


import rclpy
from rclpy.executors import MultiThreadedExecutor

__all__ = ['SprayTask']


def main():
    rclpy.init()
    node = SprayTask()
    # 使用 4 线程的 MultiThreadedExecutor：
    # 保证 Action Server 的长时循环（轴计算/等待 YOLO 推理）不会阻塞急停订阅或 TF 监听。
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
