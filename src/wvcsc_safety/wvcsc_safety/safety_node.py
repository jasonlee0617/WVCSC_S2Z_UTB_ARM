"""ROS 2 safety gate for real WVCSC navigation and controlled arm recovery.

The node is deliberately independent from the protected base and arm drivers.  It
is the sole publisher feeding the real base driver's ``/cmd_vel`` subscription.
Nav2 publishes to ``/cmd_vel_nav``.  A controlled abort cancels the mission,
publishes zero velocity immediately, stops the arm, waits for the base to settle,
then requests the existing arm HOME reset.  A physical emergency-stop never
initiates motion while it remains asserted.
"""

import math
import threading
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger

from .core import (
    HOME_LOCKED,
    RESETTING,
    RESET_FAILED,
    RUNNING,
    STOPPED_LOCKED,
    Freshness,
    base_is_stopped,
    latch_can_clear,
    velocity_allowed,
)


class SafetyGate(Node):
    """Fail-closed velocity multiplexer and controlled-abort coordinator."""

    def __init__(self):
        super().__init__('wvcsc_safety_gate')
        defaults = {
            'nav_cmd_topic': '/cmd_vel_nav',
            'base_cmd_topic': '/cmd_vel',
            'odom_topic': '/ekf_odom',
            'scan_topic': '/scan',
            'imu_topic': '/imu',
            'command_timeout_sec': 0.30,
            'odom_timeout_sec': 0.50,
            'scan_timeout_sec': 1.00,
            'imu_timeout_sec': 1.00,
            'publish_rate_hz': 20.0,
            'linear_stop_threshold': 0.03,
            'angular_stop_threshold': 0.03,
            'stable_duration_sec': 1.0,
            'reset_timeout_sec': 90.0,
            'require_scan': True,
            'require_imu': True,
            'require_camera': True,
            'camera_health_topic': '/camera/healthy',
            'camera_health_timeout_sec': 2.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self._validate_parameters()

        self._mutex = threading.RLock()
        self._callback_group = ReentrantCallbackGroup()
        self._autonomy_enabled = False
        self._emergency_stop = False
        self._stop_latched = False
        self._state = 'MANUAL_MODE'
        self._arm_state = RUNNING
        self._latest_command = Twist()
        self._freshness = Freshness()
        self._linear_x = float('inf')
        self._angular_z = float('inf')
        self._stable_since = None
        self._controlled_abort_active = False
        self._camera_healthy = False
        self._camera_health_received_at = None
        self._reset_sent_at = None
        self._mission_cancel_future = None

        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._cmd_pub = self.create_publisher(
            Twist, str(self.get_parameter('base_cmd_topic').value), 10)
        self._autonomy_pub = self.create_publisher(
            Bool, '/safety/autonomy_enabled', latched)
        self._latched_pub = self.create_publisher(
            Bool, '/safety/stop_latched', latched)
        self._state_pub = self.create_publisher(String, '/safety/state', latched)
        self._motion_pub = self.create_publisher(
            String, '/motion_control/command', 10)

        self.create_subscription(
            Twist, str(self.get_parameter('nav_cmd_topic').value),
            self._on_nav_command, 10, callback_group=self._callback_group)
        self.create_subscription(
            Odometry, str(self.get_parameter('odom_topic').value),
            self._on_odom, 20, callback_group=self._callback_group)
        self.create_subscription(
            LaserScan, str(self.get_parameter('scan_topic').value),
            self._on_scan, 10, callback_group=self._callback_group)
        self.create_subscription(
            Imu, str(self.get_parameter('imu_topic').value),
            self._on_imu, 20, callback_group=self._callback_group)
        self.create_subscription(
            Bool, '/safety/autonomy_enabled', self._on_autonomy_topic,
            latched, callback_group=self._callback_group)
        self.create_subscription(
            Bool, '/safety/emergency_stop', self._on_emergency_stop,
            latched, callback_group=self._callback_group)
        self.create_subscription(
            String, '/motion_control/state', self._on_arm_state,
            latched, callback_group=self._callback_group)
        self.create_subscription(
            Bool, str(self.get_parameter('camera_health_topic').value),
            self._on_camera_health, latched,
            callback_group=self._callback_group)

        self._mission_cancel = self.create_client(
            Trigger, '/mission/cancel', callback_group=self._callback_group)
        self.create_service(
            SetBool, '/safety/set_autonomy_enabled', self._set_autonomy,
            callback_group=self._callback_group)
        self.create_service(
            Trigger, '/safety/controlled_abort', self._controlled_abort,
            callback_group=self._callback_group)
        self.create_service(
            Trigger, '/safety/reset', self._reset_safety,
            callback_group=self._callback_group)

        period = 1.0 / float(self.get_parameter('publish_rate_hz').value)
        self.create_timer(period, self._tick, callback_group=self._callback_group)
        self._publish_state()
        self._publish_zero()

    def _validate_parameters(self):
        positive = (
            'command_timeout_sec', 'odom_timeout_sec', 'scan_timeout_sec',
            'imu_timeout_sec', 'publish_rate_hz', 'linear_stop_threshold',
            'angular_stop_threshold', 'stable_duration_sec', 'reset_timeout_sec',
            'camera_health_timeout_sec',
        )
        for name in positive:
            value = float(self.get_parameter(name).value)
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f'{name} must be finite and positive')

    @staticmethod
    def _now():
        return time.monotonic()

    def _on_nav_command(self, message):
        with self._mutex:
            self._latest_command = message
            self._freshness = Freshness(
                command=self._now(), odom=self._freshness.odom,
                scan=self._freshness.scan, imu=self._freshness.imu)

    def _on_odom(self, message):
        with self._mutex:
            self._linear_x = float(message.twist.twist.linear.x)
            self._angular_z = float(message.twist.twist.angular.z)
            self._freshness = Freshness(
                command=self._freshness.command, odom=self._now(),
                scan=self._freshness.scan, imu=self._freshness.imu)

    def _on_scan(self, _message):
        with self._mutex:
            self._freshness = Freshness(
                command=self._freshness.command, odom=self._freshness.odom,
                scan=self._now(), imu=self._freshness.imu)

    def _on_imu(self, _message):
        with self._mutex:
            self._freshness = Freshness(
                command=self._freshness.command, odom=self._freshness.odom,
                scan=self._freshness.scan, imu=self._now())

    def _on_arm_state(self, message):
        with self._mutex:
            self._arm_state = str(message.data).strip().upper()

    def _on_camera_health(self, message):
        healthy = bool(message.data)
        with self._mutex:
            was_enabled = self._autonomy_enabled
            self._camera_healthy = healthy
            self._camera_health_received_at = self._now()
        if not healthy and was_enabled:
            self._start_controlled_abort('C10 camera stream unhealthy')

    def _on_autonomy_topic(self, message):
        requested = bool(message.data)
        with self._mutex:
            previous = self._autonomy_enabled
            # 该话题同时是现场拨杆适配器的输入和安全节点的锁存
            # 状态输出。忽略自己发布的相同值，否则会形成反馈发布环。
            if requested == previous:
                return
            if requested and (self._stop_latched or self._emergency_stop):
                return
            if requested and not self._camera_ready_locked(self._now()):
                self.get_logger().warn(
                    '[SAFETY] ignored autonomy request: C10 camera is not ready')
                return
            self._autonomy_enabled = requested
        if previous and not requested:
            self._start_controlled_abort('autonomy mode disabled')
        self._publish_state()

    def _on_emergency_stop(self, message):
        active = bool(message.data)
        with self._mutex:
            self._emergency_stop = active
            if active:
                self._stop_latched = True
                self._autonomy_enabled = False
                self._controlled_abort_active = False
                self._reset_sent_at = None
                self._state = 'EMERGENCY_STOP'
        if active:
            self._publish_zero()
            self._send_motion('stop')
            self._cancel_mission()
            self.get_logger().error(
                '[SAFETY] physical emergency stop asserted; HOME motion is inhibited')
        else:
            self.get_logger().warn(
                '[SAFETY] emergency input cleared; safety remains latched')
        self._publish_state()

    def _set_autonomy(self, request, response):
        enabled = bool(request.data)
        with self._mutex:
            if enabled and (self._stop_latched or self._emergency_stop):
                response.success = False
                response.message = 'clear the safety latch before enabling autonomy'
                return response
            if enabled and not self._camera_ready_locked(self._now()):
                response.success = False
                response.message = 'C10 camera is unhealthy or its heartbeat is stale'
                return response
            previous = self._autonomy_enabled
            self._autonomy_enabled = enabled
        self._autonomy_pub.publish(Bool(data=enabled))
        if previous and not enabled:
            self._start_controlled_abort('autonomy mode disabled by operator')
        else:
            with self._mutex:
                self._state = 'AUTONOMY_READY' if enabled else 'MANUAL_MODE'
            self._publish_state()
        response.success = True
        response.message = 'autonomy enabled' if enabled else 'autonomy disabled'
        return response

    def _controlled_abort(self, _request, response):
        started = self._start_controlled_abort('controlled abort service')
        response.success = started
        response.message = (
            'controlled abort started' if started
            else 'controlled abort already active or physical emergency stop asserted')
        return response

    def _reset_safety(self, _request, response):
        with self._mutex:
            if not self._stop_latched:
                response.success = True
                response.message = 'safety latch is already clear'
                return response
            if self._emergency_stop:
                response.success = False
                response.message = 'physical emergency stop is still asserted'
                return response
            if not latch_can_clear(
                    emergency_stop=self._emergency_stop,
                    recovery_active=self._controlled_abort_active,
                    arm_state=self._arm_state):
                response.success = False
                response.message = (
                    'controlled recovery must finish at HOME_LOCKED before '
                    f'clearing safety (arm={self._arm_state})')
                return response
            self._stop_latched = False
            self._stable_since = None
            self._reset_sent_at = None
            self._state = 'MANUAL_MODE'
        self._publish_state()
        response.success = True
        response.message = (
            'safety latch cleared; explicitly resume arm, then enable autonomy')
        return response

    def _start_controlled_abort(self, reason):
        with self._mutex:
            if self._emergency_stop or self._controlled_abort_active:
                return False
            self._stop_latched = True
            self._autonomy_enabled = False
            self._controlled_abort_active = True
            self._stable_since = None
            self._reset_sent_at = None
            self._state = 'CONTROLLED_ABORT_STOPPING'
        self._autonomy_pub.publish(Bool(data=False))
        self._publish_zero()
        self._send_motion('stop')
        self._cancel_mission()
        self.get_logger().warn(f'[SAFETY] {reason}: zero velocity and arm stop sent')
        self._publish_state()
        return True

    def _cancel_mission(self):
        if self._mission_cancel.service_is_ready():
            self._mission_cancel_future = self._mission_cancel.call_async(
                Trigger.Request())
        else:
            self.get_logger().warn('[SAFETY] /mission/cancel is not ready')

    def _send_motion(self, command):
        self._motion_pub.publish(String(data=str(command)))

    def _publish_zero(self):
        self._cmd_pub.publish(Twist())

    def _effective_freshness(self):
        freshness = self._freshness
        now = self._now()
        if not bool(self.get_parameter('require_scan').value):
            freshness = Freshness(
                freshness.command, freshness.odom, now, freshness.imu)
        if not bool(self.get_parameter('require_imu').value):
            freshness = Freshness(
                freshness.command, freshness.odom, freshness.scan, now)
        return freshness

    def _camera_ready_locked(self, now):
        """Return camera interlock state while ``self._mutex`` is held."""
        if not bool(self.get_parameter('require_camera').value):
            return True
        return (
            self._camera_healthy
            and self._camera_health_received_at is not None
            and now - self._camera_health_received_at <= float(
                self.get_parameter('camera_health_timeout_sec').value)
        )

    def _tick(self):
        now = self._now()
        with self._mutex:
            camera_stale = (
                self._autonomy_enabled and not self._camera_ready_locked(now))
            allowed = velocity_allowed(
                autonomy_enabled=self._autonomy_enabled,
                stop_latched=self._stop_latched,
                emergency_stop=self._emergency_stop,
                freshness=self._effective_freshness(),
                now=now,
                command_timeout=float(self.get_parameter('command_timeout_sec').value),
                odom_timeout=float(self.get_parameter('odom_timeout_sec').value),
                scan_timeout=float(self.get_parameter('scan_timeout_sec').value),
                imu_timeout=float(self.get_parameter('imu_timeout_sec').value),
            )
            command = self._latest_command if allowed else Twist()
            controlled = self._controlled_abort_active
            emergency = self._emergency_stop
        if camera_stale:
            self._start_controlled_abort('C10 camera health heartbeat stale')
            command = Twist()
        self._cmd_pub.publish(command)
        if controlled and not emergency:
            self._advance_controlled_abort(now)

    def _advance_controlled_abort(self, now):
        with self._mutex:
            stopped = base_is_stopped(
                self._linear_x, self._angular_z,
                self.get_parameter('linear_stop_threshold').value,
                self.get_parameter('angular_stop_threshold').value)
            if not stopped:
                self._stable_since = None
                return
            if self._stable_since is None:
                self._stable_since = now
                return
            if now - self._stable_since < float(
                    self.get_parameter('stable_duration_sec').value):
                return
            if self._reset_sent_at is None:
                if self._arm_state != STOPPED_LOCKED:
                    return
                self._reset_sent_at = now
                self._state = RESETTING
                send_reset = True
            else:
                send_reset = False
                if self._arm_state == HOME_LOCKED:
                    self._controlled_abort_active = False
                    self._state = HOME_LOCKED
                    self._publish_state()
                    self.get_logger().warn(
                        '[SAFETY] controlled abort reached HOME; explicit resume required')
                    return
                if self._arm_state == RESET_FAILED or now - self._reset_sent_at > float(
                        self.get_parameter('reset_timeout_sec').value):
                    self._controlled_abort_active = False
                    self._state = RESET_FAILED
                    self._publish_state()
                    self.get_logger().error(
                        '[SAFETY] controlled HOME recovery failed; lock remains active')
                    return
        if send_reset:
            self._send_motion('reset')
            self._publish_state()
            self.get_logger().warn('[SAFETY] vehicle stable; arm reset/HOME requested')

    def _publish_state(self):
        with self._mutex:
            autonomy = self._autonomy_enabled
            latched = self._stop_latched
            state = self._state
        self._autonomy_pub.publish(Bool(data=autonomy))
        self._latched_pub.publish(Bool(data=latched))
        self._state_pub.publish(String(data=state))


def main():
    rclpy.init()
    node = SafetyGate()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node._publish_zero()
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
