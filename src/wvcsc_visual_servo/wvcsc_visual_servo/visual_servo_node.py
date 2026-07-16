import math
import threading
import time

from geometry_msgs.msg import TwistStamped
import numpy as np
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Int8, String
from std_srvs.srv import Trigger
from wvcsc_interfaces.action import AlignTarget
from wvcsc_interfaces.msg import Target2D

from .controllers.pid_controller import PIDController3D, ServoControlConfig
from .servo.command_limiter import limit_xy_norm, slew
from .servo.servo_status_policy import ServoStatusAction, ServoStatusPolicy
from .servo.target_estimator import SimpleTargetPredictor2D
from .servo.visual_servo_params import ServoRuntimeConfig


class VisualServo(Node):
    def __init__(self):
        super().__init__('wvcsc_visual_servo')
        self._declare_parameters()
        self._config = ServoRuntimeConfig.from_node(self)
        self._controller = PIDController3D(ServoControlConfig(
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
            self.get_parameter('servo_status_halt_codes').value)
        self._predictor = SimpleTargetPredictor2D()
        self._group = ReentrantCallbackGroup()
        self._lock = threading.Lock()
        self._busy = False
        self._active_mission = ''
        self._active_tree = ''
        self._active_target = ''
        self._latest = None
        self._camera = None
        self._stable_frames = 0
        self._last_command = (0.0, 0.0)
        self._servo_status = 0
        self._safety_message = ''
        self._motion_stop_sent = False

        self._twist = self.create_publisher(
            TwistStamped, str(self.get_parameter('twist_topic').value), 10)
        self._motion_command = self.create_publisher(
            String, '/motion_control/command', 10)
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
            'start_servo_service': '/servo_node/start_servo',
            'stop_servo_service': '/servo_node/stop_servo',
            'command_frame': 'camera_color_optical_frame',
            'control_rate_hz': 50.0,
            'default_timeout_sec': 8.0,
            'min_goal_timeout_sec': 0.5,
            'max_goal_timeout_sec': 30.0,
            'target_stale_timeout_sec': 0.2,
            'min_confidence': 0.70,
            'coarse_tolerance_px': 20.0,
            'fine_tolerance_px': 8.0,
            'stable_frames': 10,
            'desired_offset_u_px': 0.0,
            'desired_offset_v_px': 28.0,
            'fallback_fx': 507.872735,
            'fallback_fy': 507.872735,
            'require_camera_info': True,
            'pid_kp_xy': 0.25,
            'pid_ki_xy': 0.0,
            'pid_kd_xy': 0.01,
            'pid_d_ema_alpha': 0.65,
            'derivative_clip_xy': 2.0,
            'integral_limit_xy': 0.10,
            'max_linear_speed': 0.04,
            'max_linear_acceleration': 0.25,
            'near_target_speed_scale': 0.35,
            'warning_speed_scale': 0.35,
            'command_sign_x': 1.0,
            'command_sign_y': 1.0,
            'predict_lead_sec': 0.02,
            'max_predict_horizon_sec': 0.05,
            'zero_command_count': 5,
            'service_timeout_sec': 2.0,
            'servo_status_decel_codes': [1, 3, 6],
            'servo_status_halt_codes': [2, 4, 5],
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
        now = self._now()
        with self._lock:
            if not self._busy:
                return
            matches = (
                message.mission_id == self._active_mission
                and message.tree_id == self._active_tree
                and message.target_id == self._active_target)
            valid = (
                matches and message.valid
                and math.isfinite(message.confidence)
                and message.confidence >= self._config.min_confidence
                and message.image_width > 0 and message.image_height > 0)
            if not valid:
                self._stable_frames = 0
                self._latest = {'valid': False, 'received': now}
                self._predictor.reset()
                return
            desired_u = (
                message.image_width / 2.0 + self._config.desired_offset_u_px)
            desired_v = (
                message.image_height / 2.0 + self._config.desired_offset_v_px)
            error_u = float(message.center_u) - desired_u
            error_v = float(message.center_v) - desired_v
            if self._camera is not None:
                fx, fy = self._camera[:2]
            else:
                fx = float(self.get_parameter('fallback_fx').value)
                fy = float(self.get_parameter('fallback_fy').value)
            error = np.array([error_u / fx, error_v / fy], dtype=float)
            velocity = np.zeros(2, dtype=float)
            if self._latest is not None and self._latest.get('valid'):
                dt = now - self._latest['received']
                if 1e-3 < dt < 0.5:
                    velocity = (error - self._latest['error']) / dt
            self._predictor.update(error, velocity, now)
            if (abs(error_u) <= self._config.fine_tolerance_px
                    and abs(error_v) <= self._config.fine_tolerance_px):
                self._stable_frames += 1
            else:
                self._stable_frames = 0
            self._latest = {
                'valid': True,
                'received': now,
                'error': error,
                'error_u': error_u,
                'error_v': error_v,
                'stable_frames': self._stable_frames,
            }

    def _on_servo_status(self, message):
        decision = self._policy.decide(message.data)
        with self._lock:
            self._servo_status = int(message.data)
            if self._busy and decision.action == ServoStatusAction.HALT_RECOVERY:
                self._safety_message = decision.message
        if decision.action == ServoStatusAction.HALT_RECOVERY:
            self._publish_zero()
            self._lock_motion(decision.message)

    def _execute(self, goal_handle):
        request = goal_handle.request
        timeout = float(request.timeout) or self._config.default_timeout_sec
        started = self._now()
        with self._lock:
            self._active_mission = request.mission_id
            self._active_tree = request.tree_id
            self._active_target = request.target_id
            self._latest = None
            self._stable_frames = 0
            self._last_command = (0.0, 0.0)
            self._safety_message = ''
            self._motion_stop_sent = False
            self._predictor.reset()
            self._controller.reset()
        servo_started = False
        result = AlignTarget.Result()
        try:
            ok, message = self._call_trigger(self._start_client)
            if not ok:
                self._lock_motion(f'MoveIt Servo start failed: {message}')
                return self._abort(
                    goal_handle, result, AlignTarget.Result.BUSY,
                    f'[SAFETY] MoveIt Servo start failed: {message}')
            servo_started = True
            period = 1.0 / self._config.control_rate_hz
            last_tick = self._now()
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    self._publish_zero_count()
                    goal_handle.canceled()
                    result.success = False
                    result.error_code = AlignTarget.Result.CANCELED
                    result.message = 'visual alignment canceled'
                    return result
                now = self._now()
                with self._lock:
                    latest = dict(self._latest) if self._latest is not None else None
                    safety = self._safety_message
                    camera_ready = self._camera is not None
                    status = self._servo_status
                if safety:
                    return self._abort(
                        goal_handle, result, AlignTarget.Result.TIMEOUT,
                        f'[SAFETY] {safety}')
                if now - started >= timeout:
                    code = (
                        AlignTarget.Result.TARGET_STALE
                        if latest is None or not latest.get('valid')
                        else AlignTarget.Result.TIMEOUT)
                    return self._abort(
                        goal_handle, result, code,
                        'target unavailable/stale' if code == AlignTarget.Result.TARGET_STALE
                        else 'visual alignment timed out', latest)
                fresh = (
                    latest is not None and latest.get('valid')
                    and now - latest['received'] <= self._config.stale_timeout_sec
                    and (camera_ready or not bool(
                        self.get_parameter('require_camera_info').value)))
                if not fresh:
                    self._publish_zero()
                    self._publish_feedback(goal_handle, latest)
                    time.sleep(period)
                    continue
                if latest['stable_frames'] >= self._config.stable_frames:
                    self._publish_zero_count()
                    stopped, stop_message = self._call_trigger(self._stop_client)
                    servo_started = not stopped
                    if not stopped:
                        self._lock_motion(
                            f'MoveIt Servo stop failed: {stop_message}')
                        return self._abort(
                            goal_handle, result, AlignTarget.Result.TIMEOUT,
                            f'[SAFETY] MoveIt Servo stop failed: {stop_message}',
                            latest)
                    result.success = True
                    result.error_code = AlignTarget.Result.OK
                    result.message = 'target aligned; fixed spray distance preserved'
                    result.final_error_u = latest['error_u']
                    result.final_error_v = latest['error_v']
                    goal_handle.succeed()
                    return result
                dt = max(1e-3, min(0.05, now - last_tick))
                last_tick = now
                predicted, _velocity = self._predictor.predict_to(
                    now + self._config.predict_lead_sec,
                    self._config.max_predict_horizon_sec)
                if predicted is None:
                    predicted = latest['error']
                x, y, _z, _debug = self._controller.step(
                    [predicted[0], predicted[1], 0.0], dt)
                x *= self._config.command_sign_x
                y *= self._config.command_sign_y
                scale = 1.0
                if max(abs(latest['error_u']), abs(latest['error_v'])) <= (
                        self._config.coarse_tolerance_px):
                    scale *= self._config.near_target_speed_scale
                decision = self._policy.decide(status)
                if decision.action == ServoStatusAction.DECELERATE:
                    scale *= self._config.warning_speed_scale
                x, y = limit_xy_norm(
                    x * scale, y * scale, self._config.max_linear_speed)
                x = slew(
                    x, self._last_command[0],
                    self._config.max_linear_acceleration, dt)
                y = slew(
                    y, self._last_command[1],
                    self._config.max_linear_acceleration, dt)
                self._last_command = (x, y)
                self._publish_twist(x, y)
                self._publish_feedback(goal_handle, latest)
                time.sleep(period)
            return self._abort(
                goal_handle, result, AlignTarget.Result.CANCELED,
                'ROS shutdown during visual alignment')
        finally:
            self._publish_zero_count()
            if servo_started:
                self._call_trigger(self._stop_client)
            with self._lock:
                self._busy = False
                self._active_mission = ''
                self._active_tree = ''
                self._active_target = ''
                self._latest = None

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
        self._publish_zero_count()
        result.success = False
        result.error_code = code
        result.message = message
        if latest is not None and latest.get('valid'):
            result.final_error_u = latest['error_u']
            result.final_error_v = latest['error_v']
        goal_handle.abort()
        return result

    def _call_trigger(self, client):
        timeout = float(self.get_parameter('service_timeout_sec').value)
        if not client.wait_for_service(timeout_sec=timeout):
            return False, 'service unavailable'
        future = client.call_async(Trigger.Request())
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        if not future.done():
            return False, 'service response timed out'
        try:
            response = future.result()
        except Exception as error:
            return False, str(error)
        return bool(response.success), str(response.message)

    def _publish_twist(self, x, y):
        command = TwistStamped()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = str(self.get_parameter('command_frame').value)
        command.twist.linear.x = float(x)
        command.twist.linear.y = float(y)
        self._twist.publish(command)

    def _publish_zero(self):
        self._last_command = (0.0, 0.0)
        self._publish_twist(0.0, 0.0)

    def _publish_zero_count(self):
        for _index in range(int(self.get_parameter('zero_command_count').value)):
            self._publish_zero()
            time.sleep(0.005)

    def _lock_motion(self, reason):
        with self._lock:
            if self._motion_stop_sent:
                return
            self._motion_stop_sent = True
        command = String()
        command.data = 'stop'
        self._motion_command.publish(command)
        self.get_logger().error(f'[VISUAL_SERVO] motion locked: {reason}')

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
