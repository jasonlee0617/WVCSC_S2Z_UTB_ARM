# visual_servo_node.py
"""
病果图像平面 XY 视觉伺服 Action Server (Visual Servoing Action Server)。

节点职责：
1. 订阅 `Target2D` (YOLO发布的目标锁定信息)、C10 `CameraInfo` 和 `MoveIt Servo` 状态。
2. 在 `camera_color_optical_frame` 坐标系中计算并发布 `TwistStamped` 速度指令。
3. 通过 PID 控制器闭环控制机械臂末端相机，使目标像素锁定在指定区域。
4. 维护一个严格的稳定时间窗口（0.5s），仅当误差稳定低于 1.5px 时才向上层汇报成功。

**工程边界**：
- 仅对图像平面 X/Y 轴进行伺服，不控制 Z 轴（深度）。
- 喷洒距离由先前 `MoveIt` 规划的观察位姿保证，本节点不越权控制深度。
- 完整的安全互锁：检测到目标丢失、超时、MoveIt Servo 奇异点或安全状态时，
  立即发布零速度并调用 `stop_servo`，强制终止闭环。
"""

from dataclasses import dataclass
import math
import threading
import time

from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, JointState
from std_msgs.msg import Int8
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from trajectory_msgs.msg import JointTrajectory
from wvcsc_interfaces.action import AlignTarget
from wvcsc_interfaces.msg import Target2D
from wvcsc_interfaces.srv import ComputeSprayAim

from .aim_compensation import plane_error_mm, project_nozzle_axis
from .servo.alignment_progress import AlignmentProgress
from .servo.math_utils import (bounded_control_dt, limit_xy_norm, slew,
                                SimpleTargetPredictor2D)
from .servo.servo_status_policy import ServoStatusAction, ServoStatusPolicy
from .servo.pid_controller import PIDController2D, ServoControlConfig
from .servo.visual_servo_params import ServoRuntimeConfig


@dataclass
class _GoalState:
    """针对每个 `AlignTarget` Action Goal 的独立可变状态容器。

    由于 `VisualServo` 是单线程串行处理目标的，但为了清晰管理
    每个目标的跟踪、超时、停止等状态，将状态封装在此数据类中。
    """
    latest: dict | None = None                     # 最新接收到的视觉快照
    last_valid_target: dict | None = None          # 最后一次有效的目标快照
    target_unavailable_since: float | None = None  # 目标开始丢失的时间戳 (用于计算失联时长)
    initial_error_px: tuple | None = None          # 进入本伺服阶段时的初始误差 (调试用)
    stable_frames: int = 0                         # 当前已在最终容差区内的持续帧数
    last_command: tuple = (0.0, 0.0)               # 上一次发布的速度指令
    peak_command_norm: float = 0.0                 # 当前目标执行过程中的峰值速度 (用于诊断)
    last_control_dt: float = 0.0                   # 最近一次控制循环的 dt
    control_cycles: int = 0                        # 累计控制循环次数
    stop_code: int | None = None                   # 触发停止时的错误码
    stop_message: str = ''                         # 触发停止的错误信息
    alignment_started: float | None = None         # 本次对准开始的时间戳
    alignment_hold_latched: bool = False           # 收敛迟滞保持标志（防止误差在容差边缘反复跳动）
    initial_joint_positions: tuple = ()             # Action 开始时的机械臂关节快照
    max_joint_delta_rad: float = 0.0                # Action 期间实际最大关节位移
    servo_output_count: int = 0                     # MoveIt Servo 输出轨迹条数
    servo_output_points: int = 0                    # 最近一条输出轨迹的点数
    servo_output_velocity_count: int = 0            # 含完整速度字段的输出轨迹条数
    servo_output_first_monotonic: float | None = None
    servo_output_last_monotonic: float | None = None
    max_commanded_joint_delta_rad: float = 0.0      # Servo 轨迹相对实测关节的最大位移
    first_motion_command_monotonic: float | None = None
    direction_guard_baseline: tuple | None = None   # (wall_time, error_u_px, error_v_px)
    direction_guard_checked: bool = False


def _positive_finite_rate(node, name):
    """读取频率参数，确保其为正的有限浮点数，防止配置错误。"""
    value = float(node.get_parameter(name).value)
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError(f'{name} must be finite and positive')
    return value


def _nonnegative_finite_value(node, name):
    """读取非负有限标量，供执行链诊断阈值使用。"""
    value = float(node.get_parameter(name).value)
    if not math.isfinite(value) or value < 0.0:
        raise ValueError(f'{name} must be finite and non-negative')
    return value


def _unit_axis_sign(node, name):
    """Read an explicitly configured image-axis sign without accepting gain-like values."""
    value = float(node.get_parameter(name).value)
    if not math.isfinite(value) or value not in {-1.0, 1.0}:
        raise ValueError(f'{name} must be exactly -1.0 or 1.0')
    return value


def _unit_interval_value(node, name):
    value = float(node.get_parameter(name).value)
    if not math.isfinite(value) or not 0.0 <= value <= 1.0:
        raise ValueError(f'{name} must be finite and in [0.0, 1.0]')
    return value


def _direction_guard_result(
        baseline, latest_error, elapsed_sec, *, window_sec,
        min_axis_error_px, max_axis_growth_px):
    """Return ``None`` while sampling, ``''`` on pass, else a safe-stop reason."""
    if baseline is None or elapsed_sec < window_sec:
        return None
    _, baseline_u, baseline_v = baseline
    current_u, current_v = (float(value) for value in latest_error)
    violations = []
    for axis, initial, current in (
            ('u', float(baseline_u), current_u),
            ('v', float(baseline_v), current_v)):
        if (abs(initial) >= min_axis_error_px
                and abs(current) >= abs(initial) + max_axis_growth_px):
            violations.append(
                f'{axis}:abs_error={abs(initial):.1f}px→{abs(current):.1f}px')
    if violations:
        return '; '.join(violations)
    return ''


class VisualServo(Node):
    """以 30 Hz 闭环执行单目标 IBVS，并管理 MoveIt Servo 的安全生命周期。

    ROS 回调由 ``ReentrantCallbackGroup`` 和多线程执行器承载；共享目标、状态及命令
    受 ``_lock`` 保护。Action 执行线程负责唯一的控制循环，订阅回调只更新快照，
    从而避免视觉回调直接驱动机械臂。
    """

    _ARM_JOINT_NAMES = tuple(f'joint{index}' for index in range(1, 7))

    def __init__(self):
        super().__init__('wvcsc_visual_servo')
        self._declare_parameters()
        self._config = ServoRuntimeConfig.from_node(self)
        self._servo_response_timeout_sec = _positive_finite_rate(
            self, 'servo_response_timeout_sec')
        self._servo_min_output_rate_hz = _positive_finite_rate(
            self, 'servo_min_output_rate_hz')
        self._servo_min_commanded_joint_delta_rad = _nonnegative_finite_value(
            self, 'servo_min_commanded_joint_delta_rad')
        self._servo_min_actual_joint_delta_rad = _nonnegative_finite_value(
            self, 'servo_min_actual_joint_delta_rad')
        self._angular_u_sign = _unit_axis_sign(self, 'angular_u_sign')
        self._angular_v_sign = _unit_axis_sign(self, 'angular_v_sign')
        self._direction_guard_enabled = bool(
            self.get_parameter('direction_guard_enabled').value)
        self._direction_guard_window_sec = _positive_finite_rate(
            self, 'direction_guard_window_sec')
        self._direction_guard_min_error_px = _positive_finite_rate(
            self, 'direction_guard_min_error_px')
        self._direction_guard_max_growth_px = _positive_finite_rate(
            self, 'direction_guard_max_growth_px')
        self._direction_guard_min_confidence = _unit_interval_value(
            self, 'direction_guard_min_confidence')

        # 1. 实例化各个独立算法模块 (保持单一职责原则)
        self._controller = PIDController2D(ServoControlConfig(
            kp_xy=float(self.get_parameter('pid_kp_xy').value),
            ki_xy=float(self.get_parameter('pid_ki_xy').value),
            kd_xy=float(self.get_parameter('pid_kd_xy').value),
            d_ema_alpha=float(self.get_parameter('pid_d_ema_alpha').value),
            derivative_clip_xy=float(
                self.get_parameter('derivative_clip_xy').value),
            integral_limit_xy=float(
                self.get_parameter('integral_limit_xy').value),
        ))
        self._policy = ServoStatusPolicy(
            self.get_parameter('servo_status_decel_codes').value,
            [self.get_parameter('servo_singularity_status_code').value],
            self.get_parameter('servo_status_halt_codes').value,
            self.get_parameter('servo_status_passthrough_codes').value)
        self._predictor = SimpleTargetPredictor2D()
        self._progress = AlignmentProgress(
            self._config.fine_tolerance_px,
            self._config.stable_duration_sec,
            self._config.progress_window_sec,
            self._config.min_progress_px,
            self._config.control_resume_tolerance_px,
        )

        # 2. 多线程回调组 (允许并发执行)
        self._group = ReentrantCallbackGroup()

        # 3. 线程安全的共享锁与状态变量
        self._lock = threading.Lock()
        self._busy = False
        self._active_mission = ''
        self._active_tree = ''
        self._active_target = ''
        self._goal_state = _GoalState()
        self._camera = None
        self._aim_solution = None
        self._aim_error = ''
        self._servo_lifecycle = 'never_started'
        self._last_lifecycle_transition = ''
        self._last_service_latency_sec = 0.0
        self._servo_status = 0
        self._joint_positions = []
        self._command_mode = self._config.command_mode

        # 喷嘴外参由 URDF/robot_state_publisher 提供。每个 Action Goal 开始前
        # 使用最新 CameraInfo 和 TF 计算一次固定工距喷嘴投影；标定缺失时拒绝伺服。
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # 4. 主要 ROS 接口初始化
        self._twist = self.create_publisher(
            TwistStamped, str(self.get_parameter('twist_topic').value), 10)

        # 5. 订阅话题 (视觉输入与状态)
        self.create_subscription(
            Target2D, str(self.get_parameter('target_topic').value),
            self._on_target, qos_profile_sensor_data,
            callback_group=self._group)
        self.create_subscription(
            CameraInfo, str(self.get_parameter('camera_info_topic').value),
            self._on_camera_info, qos_profile_sensor_data,
            callback_group=self._group)
        self.create_subscription(
            Int8, str(self.get_parameter('servo_status_topic').value),
            self._on_servo_status, 10, callback_group=self._group)
        self.create_subscription(
            JointState, str(self.get_parameter('joint_state_topic').value),
            self._on_joint_state, qos_profile_sensor_data,
            callback_group=self._group)
        self.create_subscription(
            JointTrajectory,
            str(self.get_parameter('servo_command_out_topic').value),
            self._on_servo_output, 10, callback_group=self._group)

        # 6. MoveIt Servo 只在本节点首个对齐任务前启动一次。后续目标用零
        # Twist 制动，而不是在 Gazebo 组件容器中反复 pause/unpause；后者会
        # 阻塞服务响应并把一次普通对齐超时错误升级为全局安全锁定。
        self._start_client = self.create_client(
            Trigger, str(self.get_parameter('start_servo_service').value),
            callback_group=self._group)

        # SprayTask asks this service before MoveIt recentering so both stages
        # use one calibrated nozzle-axis aim pixel and one working distance.
        self._aim_service = self.create_service(
            ComputeSprayAim, str(self.get_parameter('aim_service_name').value),
            self._compute_spray_aim, callback_group=self._group)

        # 7. Action Server 接口 (供 `spray_task.py` 调用)
        self._action = ActionServer(
            self,
            AlignTarget,
            str(self.get_parameter('action_name').value),
            execute_callback=self._execute,
            goal_callback=self._goal,
            cancel_callback=self._cancel,
            callback_group=self._group,
        )

    def _declare_parameters(self):
        """声明所有 ROS2 参数（与 `visual_servo.yaml` 映射）。"""
        values = {
            'action_name': '/vision/align_target',
            'target_topic': '/vision/target',
            'camera_info_topic': '/camera/color/camera_info',
            'twist_topic': '/servo_node/delta_twist_cmds',
            'servo_status_topic': '/servo_node/status',
            'joint_state_topic': '/joint_states',
            'servo_command_out_topic': '/arm_controller/joint_trajectory',
            'start_servo_service': '/servo_node/start_servo',
            'aim_service_name': '/vision/compute_spray_aim',
            'command_frame': 'camera_color_optical_frame',
            'control_rate_hz': 30.0,
            'default_timeout_sec': 8.0,
            'min_goal_timeout_sec': 0.5,
            'max_goal_timeout_sec': 30.0,
            'target_stale_timeout_sec': 0.75,
            'target_invalid_hold_sec': 0.25,
            'min_confidence': 0.10,
            'coarse_tolerance_px': 20.0,
            'fine_tolerance_px': 1.5,
            'control_resume_tolerance_px': 2.0,
            'stable_duration_sec': 0.50,
            'progress_window_sec': 4.0,
            'min_progress_px': 1.0,
            'desired_offset_u_px': 0.0,
            'desired_offset_v_px': 0.0,
            'aim_compensation_enabled': True,
            # 工距由当前观察位和树根几何计算后随 Align Goal 传入；不得把
            # 固定标定工距误当作任务的可用范围门控。
            'aim_range_source': 'goal',
            'aim_range_min_m': 0.20,
            'aim_range_max_m': 2.00,
            'aim_nozzle_frame': 'spray_nozzle_link',
            'aim_min_forward_axis_z': 0.2,
            'aim_image_margin_px': 20.0,
            'fallback_fx': 507.872735,
            'fallback_fy': 507.872735,
            'require_camera_info': True,
            'pid_kp_xy': 4.00,
            'pid_ki_xy': 0.0,
            'pid_kd_xy': 0.005,
            'pid_d_ema_alpha': 0.65,
            'derivative_clip_xy': 2.0,
            'integral_limit_xy': 0.10,
            'max_linear_speed': 0.08,
            'max_linear_acceleration': 0.60,
            'near_target_speed_scale': 1.0,
            'near_target_control_threshold_px': 6.0,
            'near_target_control_scale': 0.35,
            'warning_speed_scale': 1.0,
            'predict_lead_sec': 0.0,
            'max_predict_horizon_sec': 0.05,
            'command_mode': 'angular_xy',
            'angular_u_sign': 1.0,
            'angular_v_sign': 1.0,
            'max_angular_speed': 0.45,
            'max_angular_acceleration': 3.00,
            'direction_guard_enabled': False,
            'direction_guard_window_sec': 1.0,
            'direction_guard_min_error_px': 20.0,
            'direction_guard_max_growth_px': 10.0,
            'direction_guard_min_confidence': 0.60,
            'servo_response_timeout_sec': 0.75,
            'servo_min_output_rate_hz': 8.0,
            'servo_min_commanded_joint_delta_rad': 0.01,
            'servo_min_actual_joint_delta_rad': 0.002,
            'zero_command_count': 8,
            'service_timeout_sec': 5.0,
            'initial_start_timeout_sec': 12.0,
            'servo_status_decel_codes': [1, 3],
            'servo_status_passthrough_codes': [6],
            'servo_singularity_status_code': 2,
            'servo_status_halt_codes': [4, 5],
        }
        for name, default in values.items():
            self.declare_parameter(name, default)

    # ---------- Action Server 生命周期回调 ----------
    def _goal(self, request):
        """对传入的 AlignTarget Goal 进行排他性校验与接受处理。"""
        timeout = float(request.timeout) or self._config.default_timeout_sec
        valid = (
            str(request.mission_id).strip()
            and str(request.tree_id).strip()
            and str(request.target_id).strip()
            and math.isfinite(timeout)
            and float(self.get_parameter('min_goal_timeout_sec').value)
            <= timeout
            <= float(self.get_parameter('max_goal_timeout_sec').value)
            and math.isfinite(float(request.working_range_m))
            and float(request.working_range_m) > 0.0
            and math.isfinite(float(request.desired_u_px))
            and math.isfinite(float(request.desired_v_px))
            and int(request.image_width) > 0
            and int(request.image_height) > 0
            and 0.0 <= float(request.desired_u_px) < int(request.image_width)
            and 0.0 <= float(request.desired_v_px) < int(request.image_height)
        )
        with self._lock:
            # 如果当前 VisualServo 正在处理上一个目标 (Busy)，则直接拒绝新 Goal
            if not valid or self._busy:
                return GoalResponse.REJECT
            self._busy = True
        return GoalResponse.ACCEPT

    @staticmethod
    def _cancel(_goal_handle):
        return CancelResponse.ACCEPT

    # ---------- 视觉与传感器回调 (只更新快照，不执行运算) ----------
    def _on_camera_info(self, message):
        if message.width <= 0 or message.height <= 0:
            return
        fx = float(message.k[0])
        fy = float(message.k[4])
        if fx > 0.0 and fy > 0.0:
            with self._lock:
                self._camera = (
                    fx, fy, float(message.k[2]), float(message.k[5]),
                    int(message.width), int(message.height))

    def _on_target(self, message):
        """将严格匹配当前 Goal 的 Target2D 分派给持有或有效处理路径。"""
        with self._lock:
            if not self._busy:
                return
            if not (message.mission_id == self._active_mission
                    and message.tree_id == self._active_tree
                    and message.target_id == self._active_target):
                return
            if (self._config.aim_compensation_enabled
                    and self._aim_solution is None):
                return
            now = self._now()
            if self._target_is_valid(message):
                self._handle_valid_target(message, now)
            else:
                self._handle_invalid_target(message, now)

    def _target_is_valid(self, message):
        return (bool(message.valid) and math.isfinite(message.confidence)
                and message.confidence >= self._config.min_confidence
                and message.image_width > 0 and message.image_height > 0)

    def _handle_invalid_target(self, message, now):
        """处理无效的目标消息，利用短暂的 Hold 时间防止因为单帧丢失而中断控制。"""
        if self._goal_state.target_unavailable_since is None:
            self._goal_state.target_unavailable_since = now
            self._predictor.reset()
        latest = self._goal_state.latest
        # 如果刚才是有效的，并且在 `invalid_target_hold_sec` (0.25s) 内，则保持之前的命令输出 0
        if (latest is not None and latest.get('valid') and
                now - latest['received'] <= self._config.invalid_target_hold_sec):
            self._goal_state.stable_frames = 0
            self._progress.reset_stable()
            self._goal_state.latest = {
                **latest,
                'hold': True,
                'stable_frames': 0,
            }
            self._publish_zero()
            return
        # 超出短暂 Hold 时间，完全清空目标状态
        self._goal_state.stable_frames = 0
        self._progress.reset_stable()
        self._goal_state.latest = {
            'valid': False,
            'received': now,
            'confidence': float(message.confidence),
            'hold': False,
        }

    def _handle_valid_target(self, message, now):
        """处理有效的目标消息，进行误差计算、预测更新和稳定状态评估。"""
        reacquired = self._goal_state.target_unavailable_since is not None
        self._goal_state.target_unavailable_since = None
        
        # 1. 计算相对喷嘴投影点的像素误差。粗重心仍只负责把目标拉入
        # Servo工作区；最终喷洒基准由喷嘴几何决定，不再硬编码28px偏移。
        desired_u, desired_v = self._desired_target_pixel(
            message.image_width, message.image_height)
        error_u = float(message.center_u) - desired_u
        error_v = float(message.center_v) - desired_v
        if self._goal_state.initial_error_px is None:
            self._goal_state.initial_error_px = (error_u, error_v)
        
        # 2. 归一化误差（转换像素误差为光学角度误差）用于 PID 计算
        if self._camera is not None:
            fx, fy = self._camera[:2]
        else:
            fx = float(self.get_parameter('fallback_fx').value)
            fy = float(self.get_parameter('fallback_fy').value)
        error = (error_u / fx, error_v / fy)
        
        # 3. 计算速度（预测器需要上一帧的误差和当前误差的差分）
        velocity = (0.0, 0.0)
        if (not reacquired and self._goal_state.latest is not None
                and self._goal_state.latest.get('valid')):
            dt = now - self._goal_state.latest['received']
            if 1e-3 < dt < 0.5:
                velocity = tuple(
                    (value - previous) / dt
                    for value, previous in zip(error, self._goal_state.latest['error']))
        self._predictor.update(error, velocity, now)
        
        # 4. 更新稳定状态 (AlignmentProgress)
        if math.hypot(error_u, error_v) <= self._config.fine_tolerance_px:
            self._goal_state.stable_frames += 1
        else:
            self._goal_state.stable_frames = 0
        if reacquired:
            self._progress.restart_progress(error_u, error_v, now)
        self._progress.update(error_u, error_v, now)
        
        # 5. 更新快照缓存，供主线程读取
        self._goal_state.latest = {
            'valid': True,
            'received': now,
            'error': error,
            'error_u': error_u,
            'error_v': error_v,
            'confidence': float(message.confidence),
            'stable_frames': self._goal_state.stable_frames,
            'hold': False,
        }
        self._goal_state.last_valid_target = dict(self._goal_state.latest)

    def _desired_target_pixel(self, image_width, image_height):
        """Return the per-goal aim pixel in the Target2D image dimensions."""
        if not self._config.aim_compensation_enabled:
            return (
                float(image_width) / 2.0 + self._config.desired_offset_u_px,
                float(image_height) / 2.0 + self._config.desired_offset_v_px,
            )
        solution = self._aim_solution
        camera = self._camera
        if solution is None or camera is None:
            raise RuntimeError('nozzle aim compensation is not ready')
        camera_width, camera_height = float(camera[4]), float(camera[5])
        return (
            solution.u_px * float(image_width) / camera_width,
            solution.v_px * float(image_height) / camera_height,
        )

    def _solve_aim_solution(self, working_range_m):
        """Compute a nozzle-axis image point without inventing a fallback."""
        working_range_m = float(working_range_m)
        if not math.isfinite(working_range_m) or working_range_m <= 0.0:
            return None, 'working range is invalid'
        if not (self._config.aim_range_min_m <= working_range_m <=
                self._config.aim_range_max_m):
            return None, (
                'working range is outside the geometric aim bounds: '
                f'requested={working_range_m:.3f}m '
                f'allowed={self._config.aim_range_min_m:.3f}-'
                f'{self._config.aim_range_max_m:.3f}m')
        with self._lock:
            camera = self._camera
        if camera is None:
            return None, 'CameraInfo unavailable'
        try:
            transform = self._tf_buffer.lookup_transform(
                str(self.get_parameter('command_frame').value),
                self._config.aim_nozzle_frame,
                Time(),
            ).transform
            solution = project_nozzle_axis(
                (
                    transform.translation.x,
                    transform.translation.y,
                    transform.translation.z,
                ),
                (
                    transform.rotation.x,
                    transform.rotation.y,
                    transform.rotation.z,
                    transform.rotation.w,
                ),
                camera,
                working_range_m,
                trim=(
                    self._config.desired_offset_u_px,
                    self._config.desired_offset_v_px,
                ),
                min_forward_axis_z=self._config.aim_min_forward_axis_z,
                image_margin_px=self._config.aim_image_margin_px,
            )
        except (TransformException, ValueError) as error:
            return None, str(error)
        return solution, ''

    def _compute_spray_aim(self, request, response):
        """Expose the same calibrated aim used by the AlignTarget action."""
        solution, message = self._solve_aim_solution(request.working_range_m)
        response.success = solution is not None
        response.message = message
        if solution is not None:
            response.desired_u_px = float(solution.u_px)
            response.desired_v_px = float(solution.v_px)
            with self._lock:
                camera = self._camera
            response.image_width = int(camera[4])
            response.image_height = int(camera[5])
        return response

    def _prepare_aim_compensation(self, working_range_m):
        """Resolve CameraInfo and camera->nozzle TF before starting Servo."""
        if not self._config.aim_compensation_enabled:
            self._aim_solution = None
            self._aim_error = ''
            return True, ''
        deadline = time.monotonic() + float(
            self.get_parameter('service_timeout_sec').value)
        last_error = 'CameraInfo unavailable'
        while rclpy.ok() and time.monotonic() < deadline:
            solution, last_error = self._solve_aim_solution(working_range_m)
            if solution is not None:
                with self._lock:
                    self._aim_solution = solution
                    self._aim_error = ''
                self.get_logger().info(
                    '[AIM] source=calibrated '
                    f'range={solution.range_m:.3f}m '
                    f'nozzle_frame={self._config.aim_nozzle_frame} '
                    f'target_pixel=({solution.u_px:.1f},{solution.v_px:.1f})')
                return True, ''
            time.sleep(0.02)
        with self._lock:
            self._aim_solution = None
            self._aim_error = last_error
        return False, last_error

    def _on_joint_state(self, message):
        positions = dict(zip(message.name, message.position))
        try:
            arm_positions = [
                float(positions[name]) for name in self._ARM_JOINT_NAMES]
        except KeyError:
            return
        with self._lock:
            self._joint_positions = arm_positions
            initial = self._goal_state.initial_joint_positions
            if self._busy and len(initial) == len(arm_positions):
                self._goal_state.max_joint_delta_rad = max(
                    self._goal_state.max_joint_delta_rad,
                    max(abs(current - start) for current, start in zip(
                        arm_positions, initial)))

    def _on_servo_output(self, message):
        """记录 Servo 是否真的向 ros2_control 发布了可执行关节轨迹。"""
        if not message.points:
            return
        with self._lock:
            if not self._busy:
                return
            observed_at = time.monotonic()
            self._goal_state.servo_output_count += 1
            self._goal_state.servo_output_points = len(message.points)
            if self._goal_state.servo_output_first_monotonic is None:
                self._goal_state.servo_output_first_monotonic = observed_at
            self._goal_state.servo_output_last_monotonic = observed_at
            current = dict(zip(self._ARM_JOINT_NAMES, self._joint_positions))
            point = message.points[0]
            if len(getattr(point, 'velocities', ())) == len(message.joint_names):
                self._goal_state.servo_output_velocity_count += 1
            if len(point.positions) != len(message.joint_names):
                return
            commanded = dict(zip(message.joint_names, point.positions))
            if all(name in commanded and name in current
                   for name in self._ARM_JOINT_NAMES):
                self._goal_state.max_commanded_joint_delta_rad = max(
                    self._goal_state.max_commanded_joint_delta_rad,
                    max(abs(float(commanded[name]) - current[name])
                        for name in self._ARM_JOINT_NAMES))

    def _servo_output_diagnostics(self):
        """Return downstream trajectory cadence without making it a motion gate."""
        with self._lock:
            state = self._goal_state
            first = state.servo_output_first_monotonic
            last = state.servo_output_last_monotonic
            count = state.servo_output_count
            velocity_count = state.servo_output_velocity_count
        rate = 0.0
        if first is not None and last is not None and last > first and count > 1:
            rate = float(count - 1) / (last - first)
        return count, velocity_count, rate

    def _servo_actuation_stall_reason(self, now):
        """Return a precise downstream-execution fault after a real command.

        A valid image target and a published Twist only prove that the Python
        controller ran.  The last leg of this chain is MoveIt Servo's
        ``JointTrajectory`` output followed by measured joint motion.  Gazebo
        previously rejected trajectories containing a non-zero terminal
        velocity while Servo status stayed ``NO_WARNING``; this watchdog turns
        that silent condition into a recoverable Action result.
        """
        with self._lock:
            state = self._goal_state
            command_started = state.first_motion_command_monotonic
            first_output = state.servo_output_first_monotonic
            last_output = state.servo_output_last_monotonic
            output_count = state.servo_output_count
            commanded_delta = state.max_commanded_joint_delta_rad
            actual_delta = state.max_joint_delta_rad

        # ``object.__new__(VisualServo)`` is intentionally used by focused
        # unit tests.  Keep the watchdog backward-compatible with those
        # minimal harnesses while real nodes always use validated parameters.
        response_timeout = getattr(self, '_servo_response_timeout_sec', 0.75)
        min_output_rate = getattr(self, '_servo_min_output_rate_hz', 8.0)
        min_commanded_delta = getattr(
            self, '_servo_min_commanded_joint_delta_rad', 0.01)
        min_actual_delta = getattr(
            self, '_servo_min_actual_joint_delta_rad', 0.002)

        if command_started is None:
            return None
        elapsed = max(0.0, now - command_started)
        if elapsed < response_timeout:
            return None
        if last_output is None:
            return (
                'no JointTrajectory received after '
                f'{elapsed:.2f}s of non-zero visual-servo command')
        output_age = max(0.0, now - last_output)
        if output_age > response_timeout:
            return (
                'JointTrajectory output stopped for '
                f'{output_age:.2f}s after visual-servo command')
        if (first_output is not None and last_output > first_output and
                output_count > 1):
            rate = float(output_count - 1) / (last_output - first_output)
            if rate < min_output_rate:
                return (
                    f'JointTrajectory output rate {rate:.1f}Hz is below '
                    f'{min_output_rate:.1f}Hz')
        if (commanded_delta >= min_commanded_delta and
                actual_delta < min_actual_delta):
            return (
                'joint state did not follow Servo trajectory '
                f'(commanded={commanded_delta:.5f}rad, '
                f'actual={actual_delta:.5f}rad)')
        return None

    def _direction_guard_decision(self, now, latest):
        """Sample the commanded image response once and stop on axis divergence.

        The active target id is already matched by ``_on_target``.  This guard
        is deliberately a one-shot real-arm safety check, not an automatic
        sign learner: ambiguous target motion must not reverse a live arm.
        """
        if not getattr(self, '_direction_guard_enabled', False):
            return None
        if (float(latest.get('confidence', 0.0))
                < getattr(self, '_direction_guard_min_confidence', 0.60)):
            return None
        error = (float(latest['error_u']), float(latest['error_v']))
        if not all(math.isfinite(value) for value in error):
            return None
        with self._lock:
            state = self._goal_state
            if state.direction_guard_checked:
                return None
            if state.direction_guard_baseline is None:
                if max(abs(value) for value in error) < (
                        getattr(self, '_direction_guard_min_error_px', 20.0)):
                    return None
                state.direction_guard_baseline = (now, *error)
                baseline = state.direction_guard_baseline
                should_log_baseline = True
            else:
                baseline = state.direction_guard_baseline
                should_log_baseline = False
            outcome = _direction_guard_result(
                baseline, error, now - baseline[0],
                window_sec=getattr(self, '_direction_guard_window_sec', 1.0),
                min_axis_error_px=getattr(
                    self, '_direction_guard_min_error_px', 20.0),
                max_axis_growth_px=getattr(
                    self, '_direction_guard_max_growth_px', 10.0))
            if outcome is not None:
                state.direction_guard_checked = True
        if should_log_baseline:
            self.get_logger().info(
                '[VISUAL_SERVO][DIRECTION_GUARD] baseline '
                f'error_px=({error[0]:.1f},{error[1]:.1f}) '
                f'window={getattr(self, "_direction_guard_window_sec", 1.0):.2f}s '
                f'u_sign={getattr(self, "_angular_u_sign", 1.0):+.0f} '
                f'v_sign={getattr(self, "_angular_v_sign", 1.0):+.0f}')
        if outcome == '':
            self.get_logger().info(
                '[VISUAL_SERVO][DIRECTION_GUARD] passed '
                f'error_px=({error[0]:.1f},{error[1]:.1f})')
        return outcome

    def _on_servo_status(self, message):
        """仲裁 MoveIt Servo 的安全状态，并将不可恢复的错误直接传递给主循环。"""
        code = int(message.data)
        decision = self._policy.decide(code)
        with self._lock:
            changed = code != self._servo_status
            self._servo_status = code
            active = self._busy
            if active and decision.action == ServoStatusAction.RECOVERABLE_STOP:
                self._goal_state.stop_code = AlignTarget.Result.SERVO_SINGULARITY
                self._goal_state.stop_message = decision.message
            elif active and decision.action == ServoStatusAction.SAFETY_STOP:
                self._goal_state.stop_code = AlignTarget.Result.SERVO_SAFETY_STOP
                self._goal_state.stop_message = decision.message
        if active and decision.action in {
                ServoStatusAction.RECOVERABLE_STOP,
                ServoStatusAction.SAFETY_STOP}:
            self._publish_zero()
        if changed and active and decision.action != ServoStatusAction.OK:
            self.get_logger().warn(
                f'[VISUAL_SERVO] Servo status={code}: {decision.message}')

    # ---------- 核心动作执行器 (The Main Control Loop) ----------
    def _execute(self, goal_handle):
        """执行一个完整的 start -> TRACK/WAIT -> zero-brake Action 生命周期。

        成功要求双轴像素误差连续保持在阈值内达到配置时长。所有退出分支共享
            ``stop_servo`` 闭包，其 ``stop_attempted`` 标记保证仅发送一次零速制动序列。
            MoveIt Servo 保持运行以接收下一颗果实的命令；其 ``incoming_command_timeout``
            仍会在命令中断时执行底层刹车。
        """
        request = goal_handle.request
        timeout = float(request.timeout) or self._config.default_timeout_sec
        started = self._now()
        with self._lock:
            self._active_mission = request.mission_id
            self._active_tree = request.tree_id
            self._active_target = request.target_id
            self._goal_state = _GoalState(
                target_unavailable_since=started,
                alignment_started=started,
                initial_joint_positions=tuple(
                    getattr(self, '_joint_positions', ())))
            self._servo_status = 0
            self._predictor.reset()
            self._controller.reset()
            self._progress.reset()
            self._aim_solution = None
            self._aim_error = ''
        servo_started = False
        brake_attempted = False
        brake_result = (True, '')
        result = AlignTarget.Result()

        def stop_servo(reason):
            """Brake with zero Twist without blocking on Servo lifecycle services."""
            nonlocal servo_started, brake_attempted, brake_result
            if brake_attempted:
                return brake_result
            if not servo_started:
                return True, ''
            brake_attempted = True
            self.get_logger().info(
                f'[VISUAL_SERVO] 零速制动 reason={reason}')
            brake_result = self._brake_servo(reason)
            stopped, stop_message = brake_result
            log = self.get_logger().info if stopped else self.get_logger().error
            log(f'[VISUAL_SERVO] 零速制动结果 success={str(stopped).lower()} '
                f'message={stop_message}')
            servo_started = False
            return brake_result

        def abort_with_stop(code, message, latest=None):
            """安全终止 Action 并停止 MoveIt Servo 的辅助函数。"""
            reason = {
                AlignTarget.Result.TIMEOUT: 'timeout',
                AlignTarget.Result.TARGET_STALE: 'target_stale',
                AlignTarget.Result.CANCELED: 'goal_canceled',
                AlignTarget.Result.SERVO_SINGULARITY: 'servo_singularity',
                AlignTarget.Result.SERVO_SAFETY_STOP: 'servo_safety_stop',
            }.get(code, 'alignment_abort')
            stopped, stop_message = stop_servo(reason)
            if not stopped:
                code = AlignTarget.Result.SERVO_SAFETY_STOP
                message = f'MoveIt Servo zero-brake failed: {stop_message}'
            return self._abort(goal_handle, result, code, message, latest)

        try:
            aim_ready, aim_message = self._prepare_aim_compensation(
                request.working_range_m)
            if not aim_ready:
                return self._abort(
                    goal_handle, result,
                    AlignTarget.Result.SERVO_SAFETY_STOP,
                    f'nozzle aim compensation unavailable: {aim_message}')
            aim = self._aim_solution
            if (self._config.aim_compensation_enabled and (aim is None or
                    int(request.image_width) != int(self._camera[4]) or
                    int(request.image_height) != int(self._camera[5]) or
                    abs(float(request.desired_u_px) - aim.u_px) > 0.5 or
                    abs(float(request.desired_v_px) - aim.v_px) > 0.5 or
                    abs(float(request.working_range_m) - aim.range_m) > 1.0e-3)):
                return self._abort(
                    goal_handle, result, AlignTarget.Result.INVALID_GOAL,
                    'AlignTarget nozzle-aim contract does not match calibrated '
                    'camera/nozzle geometry')
            ok, message = self._activate_servo()
            if not ok:
                return self._abort(
                    goal_handle, result, AlignTarget.Result.SERVO_SAFETY_STOP,
                    f'MoveIt Servo activation failed: {message}')
            servo_started = True
            period = 1.0 / self._config.control_rate_hz
            self.get_logger().info(
                f'[VISUAL_SERVO] 进入伺服 target={request.target_id} '
                f'rate={self._config.control_rate_hz:.1f}Hz '
                f'command_mode={self._command_mode} '
                f'angular_u_sign={getattr(self, "_angular_u_sign", 1.0):+.0f} '
                f'angular_v_sign={getattr(self, "_angular_v_sign", 1.0):+.0f}')
            
            # 使用单调墙钟时间进行积分步长计算，避免 Gazebo /clock 回退时控制崩溃
            last_control_tick = time.monotonic()
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    stopped, stop_message = stop_servo('goal_canceled')
                    if not stopped:
                        return self._abort(
                            goal_handle, result,
                            AlignTarget.Result.SERVO_SAFETY_STOP,
                            f'MoveIt Servo stop failed: {stop_message}')
                    goal_handle.canceled()
                    result.success = False
                    result.error_code = AlignTarget.Result.CANCELED
                    result.message = 'visual alignment canceled'
                    return result
                
                now = self._now()
                with self._lock:
                    latest = dict(self._goal_state.latest) if self._goal_state.latest is not None else None
                    stop_code = self._goal_state.stop_code
                    stop_message = self._goal_state.stop_message
                    camera_ready = self._camera is not None
                    status = self._servo_status
                    unavailable_since = self._goal_state.target_unavailable_since
                
                # 1. 安全状态仲裁
                if stop_code is not None:
                    return abort_with_stop(
                        stop_code, stop_message,
                        self._terminal_target_snapshot(now, latest))
                
                # 2. 超时检查
                if now - started >= timeout:
                    code = (
                        AlignTarget.Result.TARGET_STALE
                        if latest is None or not latest.get('valid')
                        else AlignTarget.Result.TIMEOUT)
                    return abort_with_stop(
                        code,
                        'target unavailable/stale' if code == AlignTarget.Result.TARGET_STALE
                        else 'visual alignment timed out',
                        self._terminal_target_snapshot(now, latest))
                
                # 3. 目标有效性检查 (处理短时丢失)
                target_fresh = (
                    latest is not None and latest.get('valid')
                    and not latest.get('hold')
                    and now - latest['received'] <= self._config.stale_timeout_sec)
                if not target_fresh:
                    if unavailable_since is None:
                        unavailable_since = (
                            started if latest is None
                            else float(latest.get('received', now)))
                        with self._lock:
                            if self._goal_state.target_unavailable_since is None:
                                self._goal_state.target_unavailable_since = unavailable_since
                    unavailable_duration = max(0.0, now - unavailable_since)
                    self._publish_zero()
                    self._publish_feedback(goal_handle, latest)
                    if unavailable_duration >= self._config.stale_timeout_sec:
                        return abort_with_stop(
                            AlignTarget.Result.TARGET_STALE,
                            'target continuously unavailable for '
                            f'{unavailable_duration:.2f}s',
                            self._terminal_target_snapshot(now, latest))
                    time.sleep(period)
                    continue
                
                # 4. 相机内参准备
                if not (
                        camera_ready or not bool(
                            self.get_parameter('require_camera_info').value)):
                    self._publish_zero()
                    self._publish_feedback(goal_handle, latest)
                    time.sleep(period)
                    continue

                direction_guard = self._direction_guard_decision(now, latest)
                if direction_guard:
                    return abort_with_stop(
                        AlignTarget.Result.SERVO_SAFETY_STOP,
                        'image-axis direction guard stopped Servo: '
                        f'{direction_guard}',
                        self._terminal_target_snapshot(now, latest))

                actuation_stall = self._servo_actuation_stall_reason(
                    time.monotonic())
                if actuation_stall:
                    return abort_with_stop(
                        AlignTarget.Result.SERVO_SAFETY_STOP,
                        f'servo actuation stalled: {actuation_stall}',
                        self._terminal_target_snapshot(now, latest))

                # 5. 评估对准状态 (AlignmentProgress)
                with self._lock:
                    aligned = self._progress.aligned
                    stalled = self._progress.stalled(now)
                    stable_duration = self._progress.stable_duration
                
                if aligned:
                    elapsed = max(0.0, now - started)
                    self.get_logger().info(
                        '[VISUAL_SERVO] alignment_result '
                        f'code={AlignTarget.Result.OK} message=target aligned '
                        f'elapsed={elapsed:.3f}s '
                        f'error_px=({latest["error_u"]:.2f},'
                        f'{latest["error_v"]:.2f}) '
                        f'stable_duration={stable_duration:.3f}s')
                    if self._config.aim_compensation_enabled:
                        fx, fy = self._camera[:2]
                        metric_error = plane_error_mm(
                            latest['error_u'], latest['error_v'], fx, fy,
                            aim.range_m)
                        self.get_logger().info(
                            '[AIM] aligned '
                            f'error_px=({latest["error_u"]:.1f},'
                            f'{latest["error_v"]:.1f}) '
                            f'estimated_plane_error_mm={metric_error:.1f}')
                    stopped, stop_message = stop_servo('target_aligned')
                    if not stopped:
                        return self._abort(
                            goal_handle, result,
                            AlignTarget.Result.SERVO_SAFETY_STOP,
                            f'MoveIt Servo stop failed: {stop_message}',
                            latest)
                    result.success = True
                    result.error_code = AlignTarget.Result.OK
                    result.message = (
                        'target aligned to compensated nozzle aim; '
                        'fixed spray distance preserved')
                    result.final_error_u = latest['error_u']
                    result.final_error_v = latest['error_v']
                    goal_handle.succeed()
                    return result
                
                if stalled:
                    return abort_with_stop(
                        (AlignTarget.Result.SERVO_SINGULARITY
                         if status == 6 else AlignTarget.Result.TIMEOUT),
                        ('visual alignment stalled while leaving singularity'
                         if status == 6 else 'visual alignment stalled'),
                        self._terminal_target_snapshot(now, latest))
                
                # 6. 迟滞带保持 (Schmitt-trigger 防抖动)
                error_norm = math.hypot(
                    latest['error_u'], latest['error_v'])
                (hold_alignment, self._goal_state.alignment_hold_latched) = (
                    self._alignment_hold_decision(
                        error_norm,
                        self._config.fine_tolerance_px,
                        self._config.control_resume_tolerance_px,
                        self._goal_state.alignment_hold_latched))
                if hold_alignment:
                    self._publish_zero()
                    self._publish_feedback(goal_handle, latest)
                    time.sleep(period)
                    continue
                
                # 7. 执行控制计算与命令发布
                control_now = time.monotonic()
                control_dt = max(0.0, control_now - last_control_tick)
                dt = bounded_control_dt(
                    control_dt, self._config.control_rate_hz)
                last_control_tick = control_now
                self._goal_state.last_control_dt = control_dt
                self._goal_state.control_cycles += 1
                
                # 预测 (实际配置为0，未启动预测)
                predicted, _velocity = self._predictor.predict_to(
                    now + self._config.predict_lead_sec,
                    self._config.max_predict_horizon_sec)
                if predicted is None:
                    predicted = latest['error']
                
                # PID 计算与指令合成
                x, y, _ = self._controller.step(predicted, dt)
                scale = 1.0
                if max(abs(latest['error_u']), abs(latest['error_v'])) <= (
                        self._config.coarse_tolerance_px):
                    scale *= self._config.near_target_speed_scale
                if math.hypot(
                        latest['error_u'], latest['error_v']) <= float(
                            self.get_parameter(
                                'near_target_control_threshold_px').value):
                    scale *= float(self.get_parameter(
                        'near_target_control_scale').value)
                decision = self._policy.decide(status)
                if decision.action == ServoStatusAction.DECELERATE:
                    scale *= self._config.warning_speed_scale
                speed_limit, acceleration_limit = self._command_limits()
                x, y = limit_xy_norm(x * scale, y * scale, speed_limit)
                x = slew(
                    x, self._goal_state.last_command[0], acceleration_limit, dt)
                y = slew(
                    y, self._goal_state.last_command[1], acceleration_limit, dt)
                self._goal_state.last_command = (x, y)
                self._goal_state.peak_command_norm = max(
                    self._goal_state.peak_command_norm, math.hypot(x, y))
                if math.hypot(x, y) > 1e-6:
                    with self._lock:
                        if self._goal_state.first_motion_command_monotonic is None:
                            self._goal_state.first_motion_command_monotonic = (
                                time.monotonic())
                self._publish_twist(x, y)
                self._publish_feedback(goal_handle, latest)
                time.sleep(period)
                
            return abort_with_stop(
                AlignTarget.Result.CANCELED,
                'ROS shutdown during visual alignment',
                self._terminal_target_snapshot(self._now()))
        finally:
            self._publish_zero()
            if servo_started and not brake_attempted:
                stopped, stop_message = stop_servo('execute_cleanup')
                if not stopped:
                    self.get_logger().error(
                        f'[VISUAL_SERVO] final stop failed: {stop_message}')
            with self._lock:
                self._busy = False
                self._active_mission = ''
                self._active_tree = ''
                self._active_target = ''
                self._goal_state = _GoalState()

    # ---------- 辅助函数 ----------
    def _publish_feedback(self, goal_handle, latest):
        feedback = AlignTarget.Feedback()
        if latest is not None and latest.get('valid'):
            feedback.error_u = latest['error_u']
            feedback.error_v = latest['error_v']
            feedback.stable_frames = latest['stable_frames']
            feedback.phase = (
                AlignTarget.Feedback.ALIGNED
                if latest['stable_frames'] > 0 else AlignTarget.Feedback.ACQUIRING)
        goal_handle.publish_feedback(feedback)

    def _abort(self, goal_handle, result, code, message, latest=None):
        """生成唯一的失败摘要，同时保留停止前最后一个有效目标误差。"""
        self._publish_zero()
        if latest is None:
            latest = self._terminal_target_snapshot(self._now())
        result.success = False
        result.error_code = code
        result.message = message
        if latest is not None and latest.get('valid'):
            result.final_error_u = latest['error_u']
            result.final_error_v = latest['error_v']
        goal_handle.abort()
        event = {
            AlignTarget.Result.TIMEOUT: 'timeout',
            AlignTarget.Result.TARGET_STALE: 'target_stale',
            AlignTarget.Result.CANCELED: 'canceled',
            AlignTarget.Result.SERVO_SINGULARITY: 'servo_singularity',
            AlignTarget.Result.SERVO_SAFETY_STOP: 'servo_safety_stop',
        }.get(code, 'aborted')
        age = -1.0 if latest is None else max(
            0.0, self._now() - float(latest.get('received', self._now())))
        unavailable = 0.0 if latest is None else float(
            latest.get('target_unavailable_sec', 0.0))
        (servo_outputs, servo_velocity_outputs,
         servo_output_rate_hz) = self._servo_output_diagnostics()
        self.get_logger().warn(
            f'[VISUAL_SERVO] {event} code={code} target_age={age:.2f}s '
            f'target_unavailable={unavailable:.2f}s '
            f'initial_error_px={self._goal_state.initial_error_px} '
            f'error_px=({result.final_error_u:.1f},{result.final_error_v:.1f}) '
            f'stable_frames={0 if latest is None else latest.get("stable_frames", 0)} '
            f'camera_ready={self._camera is not None} '
            f'target_hold={False if latest is None else latest.get("hold", False)} '
            f'{self._command_labels()[0]}=({self._goal_state.last_command[0]:.3f},'
            f'{self._goal_state.last_command[1]:.3f}) '
            f'peak_command_{self._command_labels()[1]}={self._goal_state.peak_command_norm:.3f} '
            f'control_cycles={self._goal_state.control_cycles} control_dt={self._goal_state.last_control_dt:.3f}s '
            f'servo_outputs={servo_outputs} '
            f'servo_velocity_outputs={servo_velocity_outputs} '
            f'servo_output_rate_hz={servo_output_rate_hz:.1f} '
            f'servo_output_points={self._goal_state.servo_output_points} '
            f'commanded_joint_delta={self._goal_state.max_commanded_joint_delta_rad:.5f}rad '
            f'actual_joint_delta={self._goal_state.max_joint_delta_rad:.5f}rad '
            f'servo_status={self._servo_status} message={message}')
        return result

    def _terminal_target_snapshot(self, now, latest=None):
        """在零速停止脉冲使目标老化前，冻结最后一个有意义的目标快照。"""
        with self._lock:
            current = (
                dict(self._goal_state.latest) if latest is None and self._goal_state.latest is not None
                else (dict(latest) if latest is not None else None))
            last_valid = (
                dict(self._goal_state.last_valid_target)
                if self._goal_state.last_valid_target is not None else None)
            unavailable_since = self._goal_state.target_unavailable_since
        snapshot = current if current is not None and current.get('valid') else last_valid
        if snapshot is None:
            snapshot = current
        if snapshot is not None:
            snapshot = dict(snapshot)
            snapshot['terminal_target_valid'] = bool(
                current is not None and current.get('valid')
                and not current.get('hold'))
            snapshot['target_unavailable_sec'] = (
                0.0 if unavailable_since is None
                else max(0.0, float(now) - float(unavailable_since)))
        return snapshot

    def _call_trigger(self, client, timeout):
        """Call a lifecycle Trigger and retain its measured round-trip time."""
        timeout = float(timeout)
        started = time.monotonic()
        if not client.wait_for_service(timeout_sec=timeout):
            self._last_service_latency_sec = time.monotonic() - started
            return False, 'service unavailable'
        future = client.call_async(Trigger.Request())
        completed = threading.Event()
        future.add_done_callback(lambda _future: completed.set())
        remaining = max(0.0, timeout - (time.monotonic() - started))
        if not completed.wait(timeout=remaining):
            self._last_service_latency_sec = time.monotonic() - started
            return False, 'service response timed out'
        self._last_service_latency_sec = time.monotonic() - started
        try:
            response = future.result()
        except Exception as error:
            return False, str(error)
        return bool(response.success), str(response.message)

    def _activate_servo(self):
        """Start Servo once; later alignment goals reuse the active Servo loop."""
        state = self._servo_lifecycle
        if state == 'running':
            return True, 'already running'
        if state == 'never_started':
            transition = 'start'
            client = self._start_client
            timeout = float(self.get_parameter('initial_start_timeout_sec').value)
        else:
            return False, f'Servo lifecycle is {state}; manual recovery required'
        self._last_lifecycle_transition = transition
        ok, message = self._call_trigger(client, timeout)
        if ok:
            self._servo_lifecycle = 'running'
        else:
            self._servo_lifecycle = 'unknown'
        return ok, message

    def _brake_servo(self, reason):
        """Publish a bounded zero-command sequence and keep Servo ready to resume."""
        self._publish_zero_count()
        if self._servo_lifecycle != 'running':
            return False, f'Servo lifecycle is {self._servo_lifecycle}; cannot brake'
        self._last_lifecycle_transition = f'zero_brake:{reason}'
        return True, 'zero commands published'

    def _command_limits(self):
        """返回当前输出空间的范数和加速度上限。"""
        if self._command_mode == 'angular_xy':
            return (
                self._config.max_angular_speed,
                self._config.max_angular_acceleration)
        return (
            self._config.max_linear_speed,
            self._config.max_linear_acceleration)

    @staticmethod
    def _alignment_hold_decision(
            error_norm_px, fine_tolerance_px,
            resume_tolerance_px, latched):
        """返回 `(hold_zero, new_latch)` 用于收敛迟滞逻辑。"""
        error = float(error_norm_px)
        fine = float(fine_tolerance_px)
        resume = float(resume_tolerance_px)
        if error <= fine:
            return True, True
        if latched and error <= resume:
            return False, True
        return False, False

    def _command_components(self, x, y):
        """把图像平面 PID 输出映射为完整的 camera optical Twist。

        optical 坐标为 +X 右、+Y 下、+Z 前。默认静止目标的 u 正误差需要相机
        向右转（+angular.y），v 正误差需要相机向下转（-angular.x）。实机可
        仅通过 ±1 的轴符号参数适配经标定后的安装方向，而不改变 PID 本身。
        """
        x, y = float(x), float(y)
        if self._command_mode == 'angular_xy':
            return (
                0.0, 0.0,
                -getattr(self, '_angular_v_sign', 1.0) * y,
                getattr(self, '_angular_u_sign', 1.0) * x)
        return x, y, 0.0, 0.0

    def _command_labels(self):
        """返回 `(log_label, unit_suffix)` 用于日志输出。"""
        if self._command_mode == 'angular_xy':
            return 'cmd_angular_rps', 'rps'
        return 'cmd_velocity_mps', 'mps'

    def _publish_twist(self, x, y):
        command = TwistStamped()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = str(self.get_parameter('command_frame').value)
        (command.twist.linear.x, command.twist.linear.y,
         command.twist.angular.x, command.twist.angular.y) = (
             self._command_components(x, y))
        self._twist.publish(command)

    def _publish_zero(self):
        self._goal_state.last_command = (0.0, 0.0)
        self._publish_twist(0.0, 0.0)

    def _publish_zero_count(self):
        count = int(self.get_parameter('zero_command_count').value)
        period = 1.0 / self._config.control_rate_hz
        for _index in range(count):
            self._publish_zero()
            time.sleep(period)

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9


def main():
    rclpy.init()
    node = VisualServo()
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
