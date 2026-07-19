"""病果图像平面 XY 视觉伺服 Action Server。

节点订阅 ``Target2D``、C10 ``CameraInfo``、MoveIt Servo 状态和机械臂关节状态，
在 ``camera_color_optical_frame`` 中发布 ``TwistStamped``。默认的仿真配置采用相机
optical X/Y 线速度；角速度模式仅保留给独立实验。IBVS 只校正图像平面，喷洒距离
由先前观察位姿保持，不在本节点控制深度。

每个 ``AlignTarget`` Goal 独占控制器。目标丢失、取消、超时、奇异点或安全状态都会
先发零速度，再且仅调用一次 ``stop_servo``。高频完整诊断发布到
``/vision/visual_servo_debug``，终端只保留限频的人类可读状态和最终结果。
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
from sensor_msgs.msg import CameraInfo, JointState
from std_msgs.msg import Int8, String
from std_srvs.srv import Trigger
from wvcsc_interfaces.action import AlignTarget
from wvcsc_interfaces.msg import Target2D

from .servo.alignment_progress import AlignmentProgress
from .servo.math_utils import (bounded_control_dt, limit_xy_norm, slew,
                                SimpleTargetPredictor2D)
from .servo.debug_snapshot import debug_json, debug_publish_due
from .servo.servo_status_policy import ServoStatusAction, ServoStatusPolicy
from .servo.pid_controller import PIDController2D, ServoControlConfig
from .servo.visual_servo_params import ServoRuntimeConfig


@dataclass
class _GoalState:
    """Per-goal mutable state reset at the start of each AlignTarget execution."""
    latest: dict | None = None
    last_valid_target: dict | None = None
    target_unavailable_since: float | None = None
    initial_error_px: tuple | None = None
    stable_frames: int = 0
    last_command: tuple = (0.0, 0.0)
    peak_command_norm: float = 0.0
    last_control_dt: float = 0.0
    control_cycles: int = 0
    stop_code: int | None = None
    stop_message: str = ''
    alignment_started: float | None = None
    last_debug_publish: float | None = None
    last_terminal_status_log: float | None = None
    last_terminal_phase: tuple | None = None
    alignment_hold_latched: bool = False


def _positive_finite_rate(node, name):
    """读取日志频率参数，并拒绝会破坏限频器的零值、NaN 和 Inf。"""
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
        self._group = ReentrantCallbackGroup()
        self._lock = threading.Lock()
        self._busy = False
        self._active_mission = ''
        self._active_tree = ''
        self._active_target = ''
        self._goal_state = _GoalState()
        self._camera = None
        self._servo_status = 0
        self._joint_positions = []
        self._command_mode = self._config.command_mode

        self._twist = self.create_publisher(
            TwistStamped, str(self.get_parameter('twist_topic').value), 10)
        self._debug = self.create_publisher(
            String, str(self.get_parameter('debug_topic').value), 10)
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
        self._start_client = self.create_client(
            Trigger, str(self.get_parameter('start_servo_service').value),
            callback_group=self._group)
        self._stop_client = self.create_client(
            Trigger, str(self.get_parameter('stop_servo_service').value),
            callback_group=self._group)
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
        values = {
            'action_name': '/vision/align_target',
            'target_topic': '/vision/target',
            'camera_info_topic': '/camera/camera/color/camera_info',
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
            # Keep direct ``ros2 run`` behavior identical to the simulation
            # profile and leave measurable margin below the strict 2 px goal.
            'fine_tolerance_px': 1.5,
            'control_resume_tolerance_px': 2.0,
            'stable_duration_sec': 0.50,
            'progress_window_sec': 4.0,
            'min_progress_px': 1.0,
            'desired_offset_u_px': 0.0,
            'desired_offset_v_px': 28.0,
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
            # The last few pixels are dominated by mask-center jitter and the
            # camera/arm response delay.  Keep the outer loop aggressive, but
            # reduce only the command gain inside this small image-space band
            # so a target does not oscillate around the strict 1.5 px gate.
            'near_target_control_threshold_px': 6.0,
            'near_target_control_scale': 0.35,
            'warning_speed_scale': 1.0,
            'predict_lead_sec': 0.0,
            'max_predict_horizon_sec': 0.05,
            # ``angular_xy`` is the eye-in-hand profile used by the Gazebo
            # spray task. ``linear_xy`` remains available for direct callers
            # and hardware experiments.
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

    def _goal(self, request):
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
            if not valid or self._busy:
                return GoalResponse.REJECT
            self._busy = True
        return GoalResponse.ACCEPT

    @staticmethod
    def _cancel(_goal_handle):
        return CancelResponse.ACCEPT

    def _on_camera_info(self, message):
        if message.width <= 0 or message.height <= 0:
            return
        fx = float(message.k[0])
        fy = float(message.k[4])
        if fx > 0.0 and fy > 0.0:
            with self._lock:
                self._camera = (fx, fy, int(message.width), int(message.height))

    def _on_target(self, message):
        """将严格匹配当前 Goal 的 Target2D 分派给持有或有效处理路径。"""
        with self._lock:
            if not self._busy:
                return
            if not (message.mission_id == self._active_mission
                    and message.tree_id == self._active_tree
                    and message.target_id == self._active_target):
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
        if self._goal_state.target_unavailable_since is None:
            self._goal_state.target_unavailable_since = now
            self._predictor.reset()
        latest = self._goal_state.latest
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
        self._goal_state.stable_frames = 0
        self._progress.reset_stable()
        self._goal_state.latest = {
            'valid': False,
            'received': now,
            'confidence': float(message.confidence),
            'hold': False,
        }

    def _handle_valid_target(self, message, now):
        reacquired = self._goal_state.target_unavailable_since is not None
        self._goal_state.target_unavailable_since = None
        desired_u = (
            message.image_width / 2.0 + self._config.desired_offset_u_px)
        desired_v = (
            message.image_height / 2.0 + self._config.desired_offset_v_px)
        error_u = float(message.center_u) - desired_u
        error_v = float(message.center_v) - desired_v
        if self._goal_state.initial_error_px is None:
            self._goal_state.initial_error_px = (error_u, error_v)
        if self._camera is not None:
            fx, fy = self._camera[:2]
        else:
            fx = float(self.get_parameter('fallback_fx').value)
            fy = float(self.get_parameter('fallback_fy').value)
        error = (error_u / fx, error_v / fy)
        velocity = (0.0, 0.0)
        if (not reacquired and self._goal_state.latest is not None
                and self._goal_state.latest.get('valid')):
            dt = now - self._goal_state.latest['received']
            if 1e-3 < dt < 0.5:
                velocity = tuple(
                    (value - previous) / dt
                    for value, previous in zip(error, self._goal_state.latest['error']))
        self._predictor.update(error, velocity, now)
        if math.hypot(error_u, error_v) <= self._config.fine_tolerance_px:
            self._goal_state.stable_frames += 1
        else:
            self._goal_state.stable_frames = 0
        if reacquired:
            self._progress.restart_progress(error_u, error_v, now)
        self._progress.update(error_u, error_v, now)
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
        servo_started = False
        stop_attempted = False
        stop_result = (True, '')
        result = AlignTarget.Result()

        def stop_servo(reason):
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
            # Use monotonic wall time for the controller integration step.  Gazebo
            # can publish several vision messages before advancing /clock, which
            # previously collapsed dt to its 1 ms fallback and made slew limiting
            # unnecessarily slow.  ROS time remains the authority for target
            # freshness and action timeouts below.
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
                if stop_code is not None:
                    return abort_with_stop(
                        stop_code, stop_message,
                        self._terminal_target_snapshot(now, latest))
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
                    stopped, stop_message = stop_servo('target_aligned')
                    if not stopped:
                        return self._abort(
                            goal_handle, result,
                            AlignTarget.Result.SERVO_SAFETY_STOP,
                            f'MoveIt Servo stop failed: {stop_message}',
                            latest)
                    result.success = True
                    result.error_code = AlignTarget.Result.OK
                    result.message = 'target aligned; fixed spray distance preserved'
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
                # Schmitt-trigger hold: after the target first enters the
                # strict radius, detector jitter in the 1.5-3 px band must not
                # restart motion. Only a real excursion beyond the outer
                # radius unlatches the hold and resumes closed-loop control.
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
                control_now = time.monotonic()
                control_dt = max(0.0, control_now - last_control_tick)
                dt = bounded_control_dt(
                    control_dt, self._config.control_rate_hz)
                last_control_tick = control_now
                self._goal_state.last_control_dt = control_dt
                self._goal_state.control_cycles += 1
                predicted, _velocity = self._predictor.predict_to(
                    now + self._config.predict_lead_sec,
                    self._config.max_predict_horizon_sec)
                if predicted is None:
                    predicted = latest['error']
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
        """Call a Servo Trigger service without recursively spinning this node.

        ``VisualServo`` is already owned by a ``MultiThreadedExecutor``.  Calling
        ``rclpy.spin_once(self)`` from its Action execution callback attempts a
        second, nested spin of the same node.  In practice the start service may
        happen to return, while the stop service can remain blocked; the Action
        then never releases ``_busy`` and every later alignment Goal is rejected.
        The executor has spare worker threads for client responses, so a short
        sleep is sufficient and keeps this wait loop deterministic.
        """
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
        """Return ``(hold_zero, new_latch)`` for convergence hysteresis."""
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
        """Return ``(log_label, unit_suffix)`` for the active command mode."""
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
        """输出低噪声终端状态；完整数值仍由 debug topic 承载。

        同一 WAIT 阶段只打印一次，WAIT 原因变化视为新阶段；WAIT -> TRACK 恢复时
        立即打印。连续 TRACK 才按 ``terminal_status_rate_hz`` 限频。
        """
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
        """发布结构稳定的高频 JSON，并区分实际线速度和角速度字段。"""
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
        payload = debug_json(
            event=event,
            mission_id=self._active_mission,
            tree_id=self._active_tree,
            target_id=self._active_target,
            elapsed_sec=0.0 if started is None else max(0.0, now - started),
            camera_ready=self._camera is not None,
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
