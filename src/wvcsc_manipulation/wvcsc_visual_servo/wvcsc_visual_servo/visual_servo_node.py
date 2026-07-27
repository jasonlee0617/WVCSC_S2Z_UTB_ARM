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

from .aim_compensation import project_nozzle_axis
from .servo.actuation_monitor import ActuationMonitor
from .servo.servo_status_policy import ServoStatusAction, ServoStatusPolicy
from .servo.servo_controller import ServoController, ServoControllerConfig
from .servo.servo_session import (
    DirectionGuardConfig,
    ServoDecisionKind,
    ServoFailureKind,
    ServoSession,
)
from .servo.visual_servo_params import ServoRuntimeConfig


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

        # 1. 每个 Goal 的纯闭环状态由 Session 持有；Node 只编排 ROS I/O。
        self._policy = ServoStatusPolicy(
            self.get_parameter('servo_status_decel_codes').value,
            [self.get_parameter('servo_singularity_status_code').value],
            self.get_parameter('servo_status_halt_codes').value,
            self.get_parameter('servo_status_passthrough_codes').value)
        controller = ServoController(ServoControllerConfig(
            control_rate_hz=self._config.control_rate_hz,
            kp_xy=float(self.get_parameter('pid_kp_xy').value),
            kd_xy=float(self.get_parameter('pid_kd_xy').value),
            d_ema_alpha=float(self.get_parameter('pid_d_ema_alpha').value),
            derivative_clip_xy=float(
                self.get_parameter('derivative_clip_xy').value),
            max_angular_speed=self._config.max_angular_speed,
            max_angular_acceleration=self._config.max_angular_acceleration,
            angular_u_sign=self._angular_u_sign,
            angular_v_sign=self._angular_v_sign,
        ))
        self._session = ServoSession(
            self._config,
            controller,
            ActuationMonitor(
                self._ARM_JOINT_NAMES,
                self._servo_response_timeout_sec,
                self._servo_min_output_rate_hz,
                self._servo_min_commanded_joint_delta_rad,
                self._servo_min_actual_joint_delta_rad,
            ),
            DirectionGuardConfig(
                self._direction_guard_enabled,
                self._direction_guard_window_sec,
                self._direction_guard_min_error_px,
                self._direction_guard_max_growth_px,
                self._direction_guard_min_confidence,
            ),
        )

        # 2. 多线程回调组 (允许并发执行)
        self._group = ReentrantCallbackGroup()

        # 3. 线程安全的共享锁与状态变量
        self._lock = threading.Lock()
        self._busy = False
        self._active_mission = ''
        self._active_tree = ''
        self._active_target = ''
        self._camera = None
        self._aim_solution = None
        self._servo_lifecycle = 'never_started'
        self._servo_status = 0

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
            'fine_tolerance_px': 1.5,
            'control_resume_tolerance_px': 2.0,
            'stable_duration_sec': 0.50,
            'progress_window_sec': 4.0,
            'min_progress_px': 1.0,
            'desired_offset_u_px': 0.0,
            'desired_offset_v_px': 0.0,
            # 工距由当前观察位和树根几何计算后随 Align Goal 传入；不得把
            # 固定标定工距误当作任务的可用范围门控。
            'aim_range_source': 'goal',
            'aim_range_min_m': 0.20,
            'aim_range_max_m': 2.00,
            'aim_nozzle_frame': 'spray_nozzle_link',
            'aim_min_forward_axis_z': 0.2,
            'aim_image_margin_px': 20.0,
            'pid_kp_xy': 4.00,
            'pid_kd_xy': 0.005,
            'pid_d_ema_alpha': 0.65,
            'derivative_clip_xy': 2.0,
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
        publish_zero = False
        with self._lock:
            if not self._busy:
                return
            if not (message.mission_id == self._active_mission
                    and message.tree_id == self._active_tree
                    and message.target_id == self._active_target):
                return
            if self._aim_solution is None:
                return
            now = self._now()
            publish_zero = self._session.observe_target(
                message, now, self._camera, self._desired_target_pixel)
        # Do not publish while holding _lock: zeroing updates Session state.
        if publish_zero:
            self._publish_zero()

    def _desired_target_pixel(self, image_width, image_height):
        """Return the per-goal aim pixel in the Target2D image dimensions."""
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
        deadline = time.monotonic() + float(
            self.get_parameter('service_timeout_sec').value)
        last_error = 'CameraInfo unavailable'
        while rclpy.ok() and time.monotonic() < deadline:
            solution, last_error = self._solve_aim_solution(working_range_m)
            if solution is not None:
                with self._lock:
                    self._aim_solution = solution
                self.get_logger().info(
                    '[AIM] source=calibrated '
                    f'range={solution.range_m:.3f}m '
                    f'nozzle_frame={self._config.aim_nozzle_frame} '
                    f'target_pixel=({solution.u_px:.1f},{solution.v_px:.1f})')
                return True, ''
            time.sleep(0.02)
        with self._lock:
            self._aim_solution = None
        return False, last_error

    def _on_joint_state(self, message):
        positions = dict(zip(message.name, message.position))
        try:
            arm_positions = [
                float(positions[name]) for name in self._ARM_JOINT_NAMES]
        except KeyError:
            return
        with self._lock:
            self._session.observe_joint_state(arm_positions, self._busy)

    def _on_servo_output(self, message):
        """记录 Servo 是否真的向 ros2_control 发布了可执行关节轨迹。"""
        with self._lock:
            self._session.observe_servo_output(
                message, self._busy, time.monotonic())

    def _on_servo_status(self, message):
        """仲裁 MoveIt Servo 的安全状态，并将不可恢复的错误直接传递给主循环。"""
        code = int(message.data)
        decision = self._policy.decide(code)
        with self._lock:
            changed = code != self._servo_status
            self._servo_status = code
            active = self._busy
            if active and decision.action == ServoStatusAction.RECOVERABLE_STOP:
                self._session.request_stop(
                    ServoFailureKind.SERVO_SINGULARITY, decision.message)
            elif active and decision.action == ServoStatusAction.SAFETY_STOP:
                self._session.request_stop(
                    ServoFailureKind.SERVO_SAFETY_STOP, decision.message)
        if active and decision.action in {
                ServoStatusAction.RECOVERABLE_STOP,
                ServoStatusAction.SAFETY_STOP}:
            self._publish_zero()
        if changed and active and decision.action != ServoStatusAction.OK:
            self.get_logger().warn(
                f'[VISUAL_SERVO] Servo status={code}: {decision.message}')

    # ---------- 核心动作执行器 (The Main Control Loop) ----------
    def _execute(self, goal_handle):
        """Apply pure Session decisions through the ROS Action lifecycle."""
        request = goal_handle.request
        timeout = float(request.timeout) or self._config.default_timeout_sec
        started = self._now()
        with self._lock:
            self._active_mission = request.mission_id
            self._active_tree = request.tree_id
            self._active_target = request.target_id
            self._session.reset(started, time.monotonic())
            self._servo_status = 0
            self._aim_solution = None
        servo_started = False
        brake_attempted = False
        brake_result = (True, '')
        result = AlignTarget.Result()

        def stop_servo(reason):
            nonlocal servo_started, brake_attempted, brake_result
            if brake_attempted:
                return brake_result
            if not servo_started:
                return True, ''
            brake_attempted = True
            self.get_logger().info(f'[VISUAL_SERVO] 零速制动 reason={reason}')
            brake_result = self._brake_servo(reason)
            stopped, stop_message = brake_result
            log = self.get_logger().info if stopped else self.get_logger().error
            log(f'[VISUAL_SERVO] 零速制动结果 success={str(stopped).lower()} '
                f'message={stop_message}')
            servo_started = False
            return brake_result

        def abort_with_stop(failure, message, latest=None):
            now = self._now()
            with self._lock:
                snapshot = self._session.terminal_snapshot(now, latest)
            reason = (
                'goal_canceled' if failure == ServoFailureKind.CANCELED
                else failure.value)
            stopped, stop_message = stop_servo(reason)
            if not stopped:
                failure = ServoFailureKind.SERVO_SAFETY_STOP
                message = f'MoveIt Servo zero-brake failed: {stop_message}'
            return self._abort(
                goal_handle, result, self._result_code(failure), message,
                failure, snapshot)

        try:
            aim_ready, aim_message = self._prepare_aim_compensation(
                request.working_range_m)
            if not aim_ready:
                return self._abort(
                    goal_handle, result, AlignTarget.Result.SERVO_SAFETY_STOP,
                    f'nozzle aim compensation unavailable: {aim_message}',
                    ServoFailureKind.SERVO_SAFETY_STOP)
            aim = self._aim_solution
            if (aim is None or
                    int(request.image_width) != int(self._camera[4]) or
                    int(request.image_height) != int(self._camera[5]) or
                    abs(float(request.desired_u_px) - aim.u_px) > 0.5 or
                    abs(float(request.desired_v_px) - aim.v_px) > 0.5 or
                    abs(float(request.working_range_m) - aim.range_m) > 1.0e-3):
                return self._abort(
                    goal_handle, result, AlignTarget.Result.INVALID_GOAL,
                    'AlignTarget nozzle-aim contract does not match calibrated '
                    'camera/nozzle geometry', ServoFailureKind.INVALID_GOAL)
            ok, message = self._activate_servo()
            if not ok:
                return self._abort(
                    goal_handle, result, AlignTarget.Result.SERVO_SAFETY_STOP,
                    f'MoveIt Servo activation failed: {message}',
                    ServoFailureKind.SERVO_SAFETY_STOP)
            servo_started = True
            period = 1.0 / self._config.control_rate_hz
            self.get_logger().info(
                f'[VISUAL_SERVO] 进入伺服 target={request.target_id} '
                f'rate={self._config.control_rate_hz:.1f}Hz '
                'command=calibrated_angular_xy '
                f'angular_u_sign={self._angular_u_sign:+.0f} '
                f'angular_v_sign={self._angular_v_sign:+.0f}')

            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    stopped, stop_message = stop_servo('goal_canceled')
                    if not stopped:
                        return self._abort(
                            goal_handle, result,
                            AlignTarget.Result.SERVO_SAFETY_STOP,
                            f'MoveIt Servo stop failed: {stop_message}',
                            ServoFailureKind.SERVO_SAFETY_STOP)
                    goal_handle.canceled()
                    result.success = False
                    result.error_code = AlignTarget.Result.CANCELED
                    result.message = 'visual alignment canceled'
                    return result

                now = self._now()
                with self._lock:
                    decision = self._session.step(
                        now, time.monotonic(), timeout,
                        self._camera is not None, self._servo_status)
                if decision.kind == ServoDecisionKind.FAIL:
                    return abort_with_stop(
                        decision.failure, decision.message, decision.latest)
                if decision.kind == ServoDecisionKind.HOLD:
                    self._publish_zero()
                    self._publish_feedback(goal_handle, decision.latest)
                    time.sleep(period)
                    continue
                if decision.kind == ServoDecisionKind.SUCCESS:
                    elapsed = max(0.0, now - started)
                    latest = decision.latest
                    self.get_logger().info(
                        '[VISUAL_SERVO] alignment_result '
                        f'code={AlignTarget.Result.OK} message=target aligned '
                        f'elapsed={elapsed:.3f}s '
                        f'error_px=({latest["error_u"]:.2f},'
                        f'{latest["error_v"]:.2f}) '
                        f'stable_duration={decision.stable_duration_sec:.3f}s')
                    stopped, stop_message = stop_servo('target_aligned')
                    if not stopped:
                        return self._abort(
                            goal_handle, result,
                            AlignTarget.Result.SERVO_SAFETY_STOP,
                            f'MoveIt Servo stop failed: {stop_message}',
                            ServoFailureKind.SERVO_SAFETY_STOP,
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
                self._publish_twist(*decision.command)
                self._publish_feedback(goal_handle, decision.latest)
                time.sleep(period)

            return abort_with_stop(
                ServoFailureKind.CANCELED,
                'ROS shutdown during visual alignment')
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
                self._session.reset(0.0, 0.0)

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

    def _abort(self, goal_handle, result, code, message, failure, snapshot=None):
        """Apply one Action failure after preserving the Session terminal report."""
        now = self._now()
        with self._lock:
            report = self._session.terminal_report(
                failure, message, now, self._servo_status,
                self._camera is not None, snapshot)
        self._publish_zero()
        result.success = False
        result.error_code = code
        result.message = message
        if report.snapshot is not None and report.snapshot.get('valid'):
            result.final_error_u = report.snapshot['error_u']
            result.final_error_v = report.snapshot['error_v']
        goal_handle.abort()
        self.get_logger().warn(f'{report.summary} code={code}')
        return result

    @staticmethod
    def _result_code(failure):
        return {
            ServoFailureKind.INVALID_GOAL: AlignTarget.Result.INVALID_GOAL,
            ServoFailureKind.TIMEOUT: AlignTarget.Result.TIMEOUT,
            ServoFailureKind.TARGET_STALE: AlignTarget.Result.TARGET_STALE,
            ServoFailureKind.CANCELED: AlignTarget.Result.CANCELED,
            ServoFailureKind.SERVO_SINGULARITY: AlignTarget.Result.SERVO_SINGULARITY,
            ServoFailureKind.SERVO_SAFETY_STOP: AlignTarget.Result.SERVO_SAFETY_STOP,
            ServoFailureKind.SERVO_ACTUATION_STALL: AlignTarget.Result.SERVO_ACTUATION_STALL,
            ServoFailureKind.SERVO_DIRECTION_DIVERGENCE:
                AlignTarget.Result.SERVO_DIRECTION_DIVERGENCE,
        }[failure]

    def _call_trigger(self, client, timeout):
        """Call a lifecycle Trigger within the caller's explicit timeout."""
        timeout = float(timeout)
        started = time.monotonic()
        if not client.wait_for_service(timeout_sec=timeout):
            return False, 'service unavailable'
        future = client.call_async(Trigger.Request())
        completed = threading.Event()
        future.add_done_callback(lambda _future: completed.set())
        remaining = max(0.0, timeout - (time.monotonic() - started))
        if not completed.wait(timeout=remaining):
            return False, 'service response timed out'
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
            client = self._start_client
            timeout = float(self.get_parameter('initial_start_timeout_sec').value)
        else:
            return False, f'Servo lifecycle is {state}; manual recovery required'
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
        return True, 'zero commands published'

    def _publish_twist(self, x, y):
        command = TwistStamped()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = str(self.get_parameter('command_frame').value)
        (command.twist.linear.x, command.twist.linear.y,
         command.twist.angular.x, command.twist.angular.y) = (
             self._session.twist_components((x, y)))
        self._twist.publish(command)

    def _publish_zero(self):
        with self._lock:
            self._session.hold_zero()
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
