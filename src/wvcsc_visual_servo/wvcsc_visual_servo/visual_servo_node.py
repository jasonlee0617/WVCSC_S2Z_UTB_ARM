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
from std_msgs.msg import Int8, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from wvcsc_interfaces.action import AlignTarget
from wvcsc_interfaces.msg import Target2D

from .aim_compensation import plane_error_mm, project_nozzle_axis
from .servo.alignment_progress import AlignmentProgress
from .servo.math_utils import (bounded_control_dt, limit_xy_norm, slew,
                                SimpleTargetPredictor2D)
from .servo.debug_snapshot import debug_json, debug_publish_due
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
    last_debug_publish: float | None = None        # 上次发布高频调试信息的时间
    last_terminal_status_log: float | None = None  # 上次打印终端人类可读日志的时间
    last_terminal_phase: tuple | None = None       # 上次记录的终端阶段状态
    alignment_hold_latched: bool = False           # 收敛迟滞保持标志（防止误差在 1.5px 附近反复跳动）


def _positive_finite_rate(node, name):
    """读取频率参数，确保其为正的有限浮点数，防止配置错误。"""
    value = float(node.get_parameter(name).value)
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError(f'{name} must be finite and positive')
    return value


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
        _positive_finite_rate(self, 'debug_rate_hz')
        _positive_finite_rate(self, 'terminal_status_rate_hz')

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
        self._debug = self.create_publisher(
            String, str(self.get_parameter('debug_topic').value), 10)

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

        # 6. MoveIt Servo 的生命周期服务客户端
        self._start_client = self.create_client(
            Trigger, str(self.get_parameter('start_servo_service').value),
            callback_group=self._group)
        self._stop_client = self.create_client(
            Trigger, str(self.get_parameter('stop_servo_service').value),
            callback_group=self._group)

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
            'debug_topic': '/vision/visual_servo_debug',
            'debug_rate_hz': 5.0,
            'terminal_status_rate_hz': 1.0,
            'start_servo_service': '/servo_node/start_servo',
            'stop_servo_service': '/servo_node/stop_servo',
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
            'aim_range_source': 'fixed',
            'aim_fixed_range_m': 1.0,
            'aim_range_tolerance_m': 0.05,
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
            'max_angular_speed': 0.45,
            'max_angular_acceleration': 3.00,
            'zero_command_count': 8,
            'service_timeout_sec': 2.0,
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

    def _prepare_aim_compensation(self):
        """Resolve CameraInfo and camera->nozzle TF before starting Servo."""
        if not self._config.aim_compensation_enabled:
            self._aim_solution = None
            self._aim_error = ''
            return True, ''
        deadline = time.monotonic() + float(
            self.get_parameter('service_timeout_sec').value)
        last_error = 'CameraInfo unavailable'
        while rclpy.ok() and time.monotonic() < deadline:
            with self._lock:
                camera = self._camera
            if camera is None:
                time.sleep(0.02)
                continue
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
                    self._config.aim_fixed_range_m,
                    trim=(
                        self._config.desired_offset_u_px,
                        self._config.desired_offset_v_px,
                    ),
                    min_forward_axis_z=self._config.aim_min_forward_axis_z,
                    image_margin_px=self._config.aim_image_margin_px,
                )
            except (TransformException, ValueError) as error:
                last_error = str(error)
                time.sleep(0.02)
                continue
            with self._lock:
                self._aim_solution = solution
                self._aim_error = ''
            self.get_logger().info(
                '[AIM] source=fixed '
                f'range={solution.range_m:.3f}m '
                f'nozzle_frame={self._config.aim_nozzle_frame}')
            self.get_logger().info(
                f'[AIM] target_pixel=({solution.u_px:.1f},'
                f'{solution.v_px:.1f}) '
                f'trim=({self._config.desired_offset_u_px:.1f},'
                f'{self._config.desired_offset_v_px:.1f})')
            return True, ''
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
        if changed:
            self._publish_debug(
                'servo_status_changed', message=decision.message, force=True)

    # ---------- 核心动作执行器 (The Main Control Loop) ----------
    def _execute(self, goal_handle):
        """执行一个完整的 start -> TRACK/WAIT -> stop Action 生命周期。

        成功要求双轴像素误差连续保持在阈值内达到配置时长。所有退出分支共享
        ``stop_servo`` 闭包，其 ``stop_attempted`` 标记保证服务最多调用一次；停止失败
        会覆盖原结果为安全停止，禁止上层误触发喷洒。
        """
        request = goal_handle.request
        timeout = float(request.timeout) or self._config.default_timeout_sec
        started = self._now()
        with self._lock:
            self._active_mission = request.mission_id
            self._active_tree = request.tree_id
            self._active_target = request.target_id
            self._goal_state = _GoalState(target_unavailable_since=started,
                                    alignment_started=started)
            self._servo_status = 0
            self._predictor.reset()
            self._controller.reset()
            self._progress.reset()
            self._aim_solution = None
            self._aim_error = ''
        servo_started = False
        stop_attempted = False
        stop_result = (True, '')
        result = AlignTarget.Result()

        def stop_servo(reason):
            """闭包：安全停止 MoveIt Servo，保证停止服务最多被调用一次。"""
            nonlocal servo_started, stop_attempted, stop_result
            if stop_attempted:
                return stop_result
            self._publish_zero_count()
            if not servo_started:
                return True, ''
            stop_attempted = True
            self.get_logger().info(
                f'[VISUAL_SERVO] 停止伺服 reason={reason}')
            stop_started = time.monotonic()
            stop_result = self._call_trigger(self._stop_client)
            stopped, stop_message = stop_result
            stop_elapsed = time.monotonic() - stop_started
            log = self.get_logger().info if stopped else self.get_logger().error
            log(
                f'[VISUAL_SERVO] 停止伺服结果 success={str(stopped).lower()} '
                f'elapsed={stop_elapsed:.3f}s message={stop_message}')
            if stopped:
                servo_started = False
            return stop_result

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
                message = f'MoveIt Servo stop failed: {stop_message}'
            return self._abort(goal_handle, result, code, message, latest)

        try:
            self._publish_debug('goal_started', force=True)
            aim_ready, aim_message = self._prepare_aim_compensation()
            if not aim_ready:
                return self._abort(
                    goal_handle, result,
                    AlignTarget.Result.SERVO_SAFETY_STOP,
                    f'nozzle aim compensation unavailable: {aim_message}')
            self._publish_debug('aim_ready', force=True)
            ok, message = self._call_trigger(self._start_client)
            if not ok:
                return self._abort(
                    goal_handle, result, AlignTarget.Result.SERVO_SAFETY_STOP,
                    f'MoveIt Servo start failed: {message}')
            servo_started = True
            self._publish_debug('servo_started', force=True)
            period = 1.0 / self._config.control_rate_hz
            self.get_logger().info(
                f'[VISUAL_SERVO] 进入伺服 target={request.target_id} '
                f'rate={self._config.control_rate_hz:.1f}Hz '
                f'command_mode={self._command_mode} '
                f'terminal_rate='
                f'{float(self.get_parameter("terminal_status_rate_hz").value):.1f}Hz')
            
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
                    self._publish_debug(
                        'canceled', result_code=result.error_code,
                        message=result.message, force=True)
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
                    self._publish_debug(
                        'target_hold' if latest is not None and latest.get('hold')
                        else 'waiting_target')
                    self._log_terminal_status(
                        'WAIT', now, latest, reason='target_unavailable')
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
                    self._publish_debug('waiting_camera_info')
                    self._log_terminal_status(
                        'WAIT', now, latest, reason='camera_info_unavailable')
                    time.sleep(period)
                    continue
                
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
                            self._config.aim_fixed_range_m)
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
                    self._publish_debug(
                        'aligned', result_code=result.error_code,
                        message=result.message, force=True)
                    return result
                
                if stalled:
                    return abort_with_stop(
                        AlignTarget.Result.TIMEOUT,
                        'visual alignment stalled',
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
                    self._publish_debug('holding_alignment')
                    self._log_terminal_status('TRACK', now, latest)
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
                x, y, _debug = self._controller.step(predicted, dt)
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
                self._publish_twist(x, y)
                self._publish_feedback(goal_handle, latest)
                self._publish_debug('control')
                self._log_terminal_status('TRACK', now, latest)
                time.sleep(period)
                
            return abort_with_stop(
                AlignTarget.Result.CANCELED,
                'ROS shutdown during visual alignment',
                self._terminal_target_snapshot(self._now()))
        finally:
            self._publish_zero()
            if servo_started and not stop_attempted:
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
            f'servo_status={self._servo_status} message={message}')
        self._publish_debug(
            event, result_code=code, message=message, force=True,
            target_snapshot=latest)
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

    def _call_trigger(self, client):
        """调用 MoveIt Servo 的 Trigger 服务（启动/停止）。"""
        timeout = float(self.get_parameter('service_timeout_sec').value)
        if not client.wait_for_service(timeout_sec=timeout):
            return False, 'service unavailable'
        future = client.call_async(Trigger.Request())
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.005)
        if not future.done():
            return False, 'service response timed out'
        try:
            response = future.result()
        except Exception as error:
            return False, str(error)
        return bool(response.success), str(response.message)

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

        optical 坐标为 +X 右、+Y 下、+Z 前。静止目标的 u 正误差需要相机向右
        转（+angular.y），v 正误差需要相机向下转（-angular.x）。这与任务层
        ``recenter_camera_pose`` 的姿态重心方向一致。
        """
        x, y = float(x), float(y)
        if self._command_mode == 'angular_xy':
            return 0.0, 0.0, -y, x
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

    def _log_terminal_status(self, phase, now, latest, reason=''):
        """输出低噪声终端状态；完整数值仍由 debug topic 承载。"""
        wall_now = time.monotonic()
        rate = float(self.get_parameter('terminal_status_rate_hz').value)
        phase = str(phase).upper()
        phase_key = (phase, reason if phase == 'WAIT' else '')
        with self._lock:
            phase_changed = phase_key != self._goal_state.last_terminal_phase
            if phase == 'WAIT' and not phase_changed:
                return
            if (phase != 'WAIT' and not phase_changed and
                    not debug_publish_due(
                        wall_now, self._goal_state.last_terminal_status_log, rate)):
                return
            self._goal_state.last_terminal_status_log = wall_now
            self._goal_state.last_terminal_phase = phase_key
            target_id = self._active_target
            started = self._goal_state.alignment_started
            command = self._goal_state.last_command
            stable_duration = self._progress.stable_duration
            servo_status = self._servo_status
        elapsed = max(0.0, now - started) if started is not None else 0.0
        confidence = 0.0 if latest is None else float(
            latest.get('confidence', 0.0))
        if not math.isfinite(confidence):
            confidence = 0.0
        if phase == 'WAIT':
            self.get_logger().info(
                f'[VISUAL_SERVO] WAIT target={target_id} elapsed={elapsed:.2f}s '
                f'error_px=n/a reason={reason or "unspecified"} '
                f'confidence={confidence:.2f}')
            return
        error_u = float(latest.get('error_u', 0.0))
        error_v = float(latest.get('error_v', 0.0))
        status_suffix = (
            '' if servo_status == 0 else
            f' servo_status={servo_status}({self._policy.status_text(servo_status)})')
        cmd_label, cmd_unit = self._command_labels()
        speed_label = f'{cmd_label}_speed'
        self.get_logger().info(
            f'[VISUAL_SERVO] TRACK target={target_id} elapsed={elapsed:.2f}s '
            f'error_px=({error_u:.1f},{error_v:.1f}) '
            f'norm_px={math.hypot(error_u, error_v):.1f} '
            f'{cmd_label}=({command[0]:.3f},{command[1]:.3f}) '
            f'{speed_label}={math.hypot(*command):.3f} '
            f'confidence={confidence:.2f} '
            f'stable_duration={stable_duration:.2f}s{status_suffix}')

    def _publish_debug(
            self, event, result_code=-1, message='', force=False,
            target_snapshot=None):
        """发布结构稳定的高频 JSON 调试数据。"""
        wall_now = time.monotonic()
        rate = float(self.get_parameter('debug_rate_hz').value)
        if not debug_publish_due(
                wall_now, self._goal_state.last_debug_publish, rate, force):
            return
        now = self._now()
        latest = (dict(target_snapshot) if target_snapshot is not None
                  else self._terminal_target_snapshot(now))
        with self._lock:
            self._goal_state.last_debug_publish = wall_now
            last_valid = self._goal_state.last_valid_target
            started = self._goal_state.alignment_started
            command = self._goal_state.last_command
            status = self._servo_status
            progress_stalled = bool(
                latest is not None and latest.get('valid')
                and not latest.get('hold') and self._progress.stalled(now))
            stable_duration = self._progress.stable_duration
        confidence = 0.0 if latest is None else float(
            latest.get('confidence', 0.0))
        if not math.isfinite(confidence):
            confidence = 0.0
        linear_x, linear_y, angular_x, angular_y = (
            self._command_components(*command))
        aim = self._aim_solution
        estimated_error = 0.0
        if (aim is not None and latest is not None and latest.get('valid')
                and self._camera is not None):
            estimated_error = plane_error_mm(
                latest.get('error_u', 0.0), latest.get('error_v', 0.0),
                self._camera[0], self._camera[1], aim.range_m)
            if not math.isfinite(estimated_error):
                estimated_error = 0.0
        payload = debug_json(
            event=event,
            mission_id=self._active_mission,
            tree_id=self._active_tree,
            target_id=self._active_target,
            elapsed_sec=0.0 if started is None else max(0.0, now - started),
            camera_ready=self._camera is not None,
            aim_compensation_enabled=self._config.aim_compensation_enabled,
            aim_ready=(
                aim is not None or not self._config.aim_compensation_enabled),
            aim_range_m=(0.0 if aim is None else aim.range_m),
            aim_u_px=(0.0 if aim is None else aim.u_px),
            aim_v_px=(0.0 if aim is None else aim.v_px),
            estimated_plane_error_mm=estimated_error,
            target_valid=bool(
                latest is not None and latest.get(
                    'terminal_target_valid', latest.get('valid'))),
            target_age_sec=(
                -1.0 if latest is None else
                max(0.0, now - float(latest['received']))),
            target_unavailable_sec=float(
                latest.get('target_unavailable_sec', 0.0)
                if latest is not None else 0.0),
            confidence=confidence,
            error_u_px=(
                0.0 if latest is None else float(latest.get('error_u', 0.0))),
            error_v_px=(
                0.0 if latest is None else float(latest.get('error_v', 0.0))),
            last_valid_error_u_px=(
                0.0 if last_valid is None
                else float(last_valid.get('error_u', 0.0))),
            last_valid_error_v_px=(
                0.0 if last_valid is None
                else float(last_valid.get('error_v', 0.0))),
            stable_frames=(
                0 if latest is None else int(latest.get('stable_frames', 0))),
            stable_duration_sec=float(stable_duration),
            progress_stalled=bool(progress_stalled),
            command_mode=self._command_mode,
            command_x_mps=linear_x,
            command_y_mps=linear_y,
            command_angular_x_rps=angular_x,
            command_angular_y_rps=angular_y,
            control_dt_sec=float(self._goal_state.last_control_dt),
            servo_status=status,
            servo_status_text=self._policy.status_text(status),
            joint_positions=list(self._joint_positions),
            result_code=int(result_code),
            message=message,
        )
        debug_message = String()
        debug_message.data = payload
        self._debug.publish(debug_message)

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
