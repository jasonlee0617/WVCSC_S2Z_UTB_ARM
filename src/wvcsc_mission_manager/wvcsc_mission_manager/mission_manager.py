# mission_manager.py
# ============================================================================
# 地面任务编排节点 (ROS2 Node)
# ============================================================================
#
# 职责：
# 1. 接收 Qt/RViz 通过 `/mission/load_manual` 提交的人工任务。
# 2. 使用 `MissionCore` 驱动状态机。
# 3. 依次调用 Nav2 (`NavigateToPose`)、停稳检测 (`StopDetector`) 和
#    机械臂喷洒 (`ExecuteSpray`)。
# 4. 处理超时、取消、返回 HOME 和状态发布。
# 5. 提供 Qt 与键盘可调用的任务服务接口 (`/mission/start`, `/mission/pause`, 等)。
#
# 注意：
# 树级顺序、超时、跳过由该节点处理；单颗树内部的病果识别和喷洒精度由
# `wvcsc_spray_task` 负责。二者严格分层。
#

import math

from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from wvcsc_interfaces.action import ExecuteSpray
from wvcsc_interfaces.msg import (
    MissionPlan,
    MissionPointPlan,
    MissionPointStatus,
    MissionStatus,
)
from wvcsc_interfaces.srv import LoadManualMission

from .core import (
    DEFAULT_ARM_BASE_FORWARD_OFFSET,
    DEFAULT_ARM_BASE_LEFT_OFFSET,
    MissionCore,
    MissionState,
)
from .mission_request import validate_manual_request
from .relay_controller import RelayController
from .stop_detector import StopDetector


class MissionManager(Node):
    """串联 Qt/RViz 人工任务、Nav2 与机械臂 Action 的事件驱动状态机外壳。"""

    def __init__(self, **kwargs):
        super().__init__('mission_manager', **kwargs)
        self._declare_parameters()
        self.core = MissionCore()
        self._map_frame = str(self.get_parameter('map_frame').value)
        self._arm_base_forward_offset = float(
            self.get_parameter('arm_base_forward_offset_m').value)
        self._arm_base_left_offset = float(
            self.get_parameter('arm_base_left_offset_m').value)
        self._arm_base_yaw = float(
            self.get_parameter('arm_base_yaw_rad').value)
        self._nav_timeout = float(self.get_parameter('nav_goal_timeout_sec').value)
        self._nav_startup_retry_timeout = float(
            self.get_parameter('nav_startup_retry_timeout_sec').value)
        self._nav_startup_retry_interval = float(
            self.get_parameter('nav_startup_retry_interval_sec').value)
        self._spray_timeout = float(self.get_parameter('spray_goal_timeout_sec').value)
        self._spray_progress_timeout = float(
            self.get_parameter('spray_progress_timeout_sec').value)
        self._wide_relay_channel = self._positive_channel('wide_relay_channel')
        self._arm_relay_channel = self._positive_channel('arm_relay_channel')
        self._require_relay_service = bool(
            self.get_parameter('require_relay_service').value)
        if self._wide_relay_channel == self._arm_relay_channel:
            raise ValueError('wide_relay_channel and arm_relay_channel must differ')
        self._wide_motion_linear_threshold = float(
            self.get_parameter('wide_spray_motion_linear_threshold').value)
        self._wide_motion_timeout = float(
            self.get_parameter('wide_spray_motion_timeout_sec').value)
        self._linear_stop_threshold = float(
            self.get_parameter('linear_stop_threshold').value)
        self._angular_stop_threshold = float(
            self.get_parameter('angular_stop_threshold').value)
        self._stop_stable_duration = float(
            self.get_parameter('stop_stable_duration_sec').value)
        self._transit_stop_stable_duration = float(
            self.get_parameter('transit_stop_stable_duration_sec').value)
        self._odom_stale_timeout = float(
            self.get_parameter('odom_stale_timeout_sec').value)
        self._return_home_after_mission = bool(
            self.get_parameter('return_home_after_mission').value)
        self._home_pose = (
            float(self.get_parameter('home_x').value),
            float(self.get_parameter('home_y').value),
            float(self.get_parameter('home_yaw').value),
        )
        if not all(math.isfinite(value) for value in self._home_pose):
            raise ValueError('home pose must contain finite values')
        if not all(math.isfinite(value) for value in (
                self._arm_base_forward_offset, self._arm_base_left_offset,
                self._arm_base_yaw, self._wide_motion_linear_threshold,
                self._wide_motion_timeout, self._linear_stop_threshold,
                self._angular_stop_threshold, self._stop_stable_duration,
                self._transit_stop_stable_duration,
                self._odom_stale_timeout)):
            raise ValueError('arm base and wide spray parameters must be finite')
        if (self._wide_motion_linear_threshold < 0.0 or
                self._wide_motion_timeout <= 0.0 or
                self._linear_stop_threshold < 0.0 or
                self._angular_stop_threshold < 0.0 or
                self._stop_stable_duration <= 0.0 or
                self._transit_stop_stable_duration < 0.0 or
                self._odom_stale_timeout <= 0.0):
            raise ValueError('wide spray motion thresholds are invalid')
        if (not math.isfinite(self._nav_startup_retry_timeout) or
                not math.isfinite(self._nav_startup_retry_interval) or
                self._nav_startup_retry_timeout <= 0.0 or
                self._nav_startup_retry_interval <= 0.0):
            raise ValueError('initial Nav2 retry timing must be finite and positive')
        self._configured_return_home_after_mission = (
            self._return_home_after_mission)
        self._configured_home_pose = self._home_pose
        self._stop_detector = StopDetector(
            self.get_parameter('linear_stop_threshold').value,
            self.get_parameter('angular_stop_threshold').value,
            self.get_parameter('stop_stable_duration_sec').value,
            self.get_parameter('odom_stale_timeout_sec').value,
            self.get_parameter('stop_verify_timeout_sec').value,
        )

        # 使用 Transient Local 持久化 QoS。
        # 这确保后启动的 Qt 或 RViz 能够立即读取到当前的任务状态，
        # 而不会因为错过初始消息而显示为空。
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._status_pub = self.create_publisher(
            MissionStatus, '/mission/status', latched)
        self._plan_pub = self.create_publisher(
            MissionPlan, '/mission/plan', latched)
        # 订阅里程计；任务仅通过 /mission/load_manual 注入。
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        # 初始化 Action 客户端
        self._nav_client = ActionClient(
            self, NavigateToPose, str(self.get_parameter('nav_action_name').value))
        self._spray_client = ActionClient(
            self, ExecuteSpray, str(self.get_parameter('spray_action_name').value))
        self._motion_command_pub = self.create_publisher(
            String, '/motion_control/command', 10)
        self._relay = RelayController(
            self,
            service_name=self.get_parameter('relay_service_name').value,
            wide_channel=self._wide_relay_channel,
            arm_channel=self._arm_relay_channel,
            require_service=self._require_relay_service,
            status_qos=latched,
            required_failure_callback=self._relay_required_failure,
        )

        # Action 状态句柄
        self._nav_handle = None
        self._spray_handle = None
        self._nav_pending = False
        self._spray_pending = False
        self._phase_started = None
        self._initial_nav_started = None
        self._nav_retry_due = None
        self._spray_last_progress = None
        self._manual_return_home = False
        self._last_odom_at = None
        self._latest_linear_speed = 0.0
        self._latest_angular_speed = 0.0
        self._wide_motion_pending = False
        self._wide_motion_deadline = None
        self._wide_stop_started_at = None
        self._wide_stop_off_pending = False
        self._abort_and_home_requested = False
        self._abort_reset_sent = False
        self._abort_reset_started = False
        self._motion_control_state = ''
        self._recovery_return_home = False
        self._nav_timeout_canceling = False
        self._nav_timeout_cancel_deadline = None

        # 对外暴露的高层操作服务
        self.create_service(Trigger, '/mission/start', self._start)
        self.create_service(Trigger, '/mission/pause', self._pause)
        self.create_service(Trigger, '/mission/resume', self._resume)
        self.create_service(
            Trigger, '/mission/skip_current', self._skip_current)
        self.create_service(
            Trigger, '/mission/return_home', self._return_home)
        self.create_service(Trigger, '/mission/cancel', self._cancel)
        self.create_service(
            Trigger, '/mission/abort_and_home', self._abort_and_home)
        self.create_service(Trigger, '/mission/reset', self._reset)
        self.create_service(
            LoadManualMission, '/mission/load_manual', self._load_manual)
        self.create_subscription(
            String, '/motion_control/state', self._on_motion_control_state, latched)

        # 调度定时器 (100ms 的高频看门狗，用于超时判断和停稳检测)
        self.create_timer(0.1, self._tick)
        self.create_timer(0.5, self._publish_status)
        self._publish_status()
        self._publish_plan()

    def _declare_parameters(self):
        """声明 YAML 中定义的各项参数。"""
        parameters = {
            'map_frame': 'map',
            'nav_action_name': '/navigate_to_pose',
            'spray_action_name': '/arm/execute_spray',
            'arm_base_forward_offset_m': DEFAULT_ARM_BASE_FORWARD_OFFSET,
            'arm_base_left_offset_m': DEFAULT_ARM_BASE_LEFT_OFFSET,
            'arm_base_yaw_rad': 0.0,
            'nav_goal_timeout_sec': 120.0,
            'nav_startup_retry_timeout_sec': 30.0,
            'nav_startup_retry_interval_sec': 0.5,
            'spray_goal_timeout_sec': 180.0,
            'spray_progress_timeout_sec': 40.0,
            'relay_service_name': '/relay/set',
            # Route progression may only depend on the relay in production.
            # Unit-test harnesses and isolated Nav2 diagnostics keep this off.
            'require_relay_service': False,
            'wide_relay_channel': 1,
            'arm_relay_channel': 2,
            'wide_spray_motion_linear_threshold': 0.03,
            'wide_spray_motion_timeout_sec': 10.0,
            'return_home_after_mission': False,
            'home_x': 0.0,
            'home_y': 0.0,
            'home_yaw': 0.0,
            'linear_stop_threshold': 0.03,
            'angular_stop_threshold': 0.03,
            'stop_stable_duration_sec': 1.0,
            'transit_stop_stable_duration_sec': 0.2,
            'odom_stale_timeout_sec': 1.0,
            'stop_verify_timeout_sec': 5.0,
            # Qt 实车路线可包含 23 株病株检查点及通行 Point；仍保留
            # 有限上界防止错误任务文件无限膨胀。
            'max_points': 64,
            'max_abs_coordinate': 50.0,
            'min_spray_duration': 0.2,
            'max_spray_duration': 10.0,
        }
        for name, default in parameters.items():
            self.declare_parameter(name, default)

    def _positive_channel(self, parameter):
        value = int(self.get_parameter(parameter).value)
        if not 1 <= value <= 255:
            raise ValueError(f'{parameter} must be within 1..255')
        return value

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _load_manual(self, request, response):
        """处理 Qt/RViz 保存的人工任务；加载本身永不隐式启动。"""
        if self._abort_and_home_requested:
            return self._reply(
                response, False,
                'abort and HOME reset is still in progress')
        try:
            points, home_pose = self._validate_manual_request(request)
            outcome = self.core.load(request.mission_id.strip(), points)
        except ValueError as error:
            self.get_logger().error(f'[MISSION] rejected manual task list: {error}')
            return self._reply(response, False, str(error))
        if outcome != 'accepted':
            message = (
                'duplicate mission id' if outcome == 'duplicate'
                else 'mission manager is busy')
            return self._reply(response, False, message)
        self._home_pose = home_pose
        self._return_home_after_mission = bool(
            request.return_home_after_mission)
        self._manual_return_home = False
        self._recovery_return_home = False
        self._nav_timeout_canceling = False
        self._nav_timeout_cancel_deadline = None
        self._clear_nav_startup_retry()
        self.get_logger().info(
            f'[MISSION] accepted manual mission={self.core.mission_id} '
            f'points={len(self.core.points)}')
        self._publish_plan()
        self._publish_status()
        return self._reply(response, True, 'manual mission loaded')

    def _validate_manual_request(self, request):
        """Bind ROS parameters to the pure Qt/RViz task contract validator."""
        return validate_manual_request(
            request,
            map_frame=self._map_frame,
            max_points=self.get_parameter('max_points').value,
            max_abs_coordinate=self.get_parameter('max_abs_coordinate').value,
            min_spray_duration=self.get_parameter('min_spray_duration').value,
            max_spray_duration=self.get_parameter('max_spray_duration').value,
            arm_base_forward_offset=self._arm_base_forward_offset,
            arm_base_left_offset=self._arm_base_left_offset,
            arm_base_yaw=self._arm_base_yaw,
        )

    def _restore_configured_mission_options(self):
        self._home_pose = self._configured_home_pose
        self._return_home_after_mission = (
            self._configured_return_home_after_mission)

    # ---------- Service Callbacks (开始、暂停、取消、跳过、复位) ----------
    def _start(self, _request, response):
        if self.core.state != MissionState.READY:
            return self._reply(response, False, 'mission is not READY')
        if self._abort_and_home_requested:
            return self._reply(
                response, False,
                'abort and HOME reset is still in progress')
        if not self._servers_ready():
            if any(point.requires_arm for point in self.core.points):
                if self._motion_control_state != 'RUNNING':
                    return self._reply(
                        response, False,
                        'arm HOME/reset is not complete; wait for RUNNING')
            return self._reply(
                response, False,
                'required Nav2, spray Action, or relay service is not ready')
        self._begin_mission_navigation()
        return self._reply(response, True, 'mission started')

    def _pause(self, _request, response):
        if not self.core.pause():
            return self._reply(response, False, 'pause is only allowed while navigating')
        self._cancel_nav_goal()
        self._publish_status()
        return self._reply(response, True, 'navigation pause requested')

    def _resume(self, _request, response):
        if self.core.state != MissionState.PAUSED:
            return self._reply(response, False, 'mission is not PAUSED')
        if self._nav_handle is not None or self._nav_pending:
            return self._reply(response, False, 'previous navigation goal is still canceling')
        if not self._nav_client.server_is_ready():
            return self._reply(response, False, 'Nav2 Action server is not ready')
        self.core.resume(returning_home=self._recovery_return_home)
        self._recovery_return_home = False
        self._start_navigation()
        return self._reply(response, True, 'mission resumed')

    def _cancel(self, _request, response):
        if not self.core.cancel():
            return self._reply(response, False, 'mission cannot be canceled in this state')
        self._stop_detector.stop()
        self._cancel_nav_goal()
        self._cancel_spray_goal()
        self._command_all_relays_off()
        self._publish_status()
        return self._reply(response, True, 'mission canceled')

    def _abort_and_home(self, _request, response):
        """Cancel the route, then request a controlled MoveIt HOME reset.

        ``motion_control`` owns the physical arm reset.  This node only
        sequences its request after the Nav2 and arm Actions have settled.
        """
        if self._abort_and_home_requested:
            if self._motion_control_state == 'RESET_FAILED':
                self._abort_reset_sent = False
                self._advance_abort_and_home()
                return self._reply(response, True, 'arm HOME reset retry requested')
            return self._reply(response, True, 'abort and HOME reset is already in progress')
        if self.core.state in self.core.ACTIVE | {MissionState.READY}:
            self.core.cancel()
        self._abort_and_home_requested = True
        self._abort_reset_sent = False
        self._abort_reset_started = False
        self._stop_detector.stop()
        self._clear_nav_startup_retry()
        self._cancel_nav_goal()
        self._cancel_spray_goal()
        self._command_all_relays_off()
        self._publish_motion_command('stop')
        prior_error = self.core.last_error
        self.core.last_error = (
            f'{prior_error}; abort requested; waiting for active actions before HOME reset'
            if prior_error else
            'abort requested; waiting for active actions before HOME reset')
        self._publish_status()
        self._advance_abort_and_home()
        return self._reply(response, True, 'abort requested; arm HOME will start after actions settle')

    def _skip_current(self, _request, response):
        if self.core.state == MissionState.PAUSED and (
                self._nav_handle is not None or self._nav_pending):
            return self._reply(
                response, False, 'wait for paused navigation goal to settle')
        point = self.core.current_point
        if point is None or not self.core.skip_current(
                self._return_home_after_mission):
            return self._reply(
                response, False,
                'skip is allowed only while READY, PAUSED, or verifying stop')
        self._stop_detector.stop()
        self.get_logger().info(f'[MISSION] skipped point={point.point_id}')
        if self.core.state in (
                MissionState.NAVIGATING, MissionState.RETURNING_HOME):
            if not self._nav_client.server_is_ready():
                self._fail('Nav2 Action server is not ready after skip')
                return self._reply(response, False, self.core.last_error)
            self._start_navigation()
        else:
            self._publish_status()
        return self._reply(response, True, f'skipped {point.point_id}')

    def _return_home(self, _request, response):
        if (self._nav_handle is not None or self._spray_handle is not None or
                self._nav_pending or self._spray_pending):
            return self._reply(response, False, 'active goal has not settled')
        if not self._nav_client.server_is_ready():
            return self._reply(response, False, 'Nav2 Action server is not ready')
        completed = self.core.state == MissionState.MISSION_COMPLETED
        if not self.core.return_home():
            return self._reply(
                response, False,
                'return home is allowed only while READY, PAUSED, verifying stop, or completed')
        self._manual_return_home = not completed
        self._stop_detector.stop()
        self._start_navigation()
        return self._reply(response, True, 'return home started')

    def _reset(self, _request, response):
        if self._abort_and_home_requested:
            return self._reply(
                response, False,
                'abort and HOME reset is still in progress')
        if (self._nav_handle is not None or self._spray_handle is not None or
                self._nav_pending or self._spray_pending):
            return self._reply(response, False, 'active goal has not settled')
        if not self.core.reset():
            return self._reply(response, False, 'reset requires a terminal state')
        self._restore_configured_mission_options()
        self._manual_return_home = False
        self._recovery_return_home = False
        self._nav_timeout_canceling = False
        self._nav_timeout_cancel_deadline = None
        self._relay.reset_failure_latch()
        self._clear_nav_startup_retry()
        self._publish_plan()
        self._publish_status()
        return self._reply(response, True, 'mission reset')

    @staticmethod
    def _reply(response, success, message):
        response.success = success
        response.message = message
        return response

    def _servers_ready(self):
        needs_spray = any(point.requires_arm for point in self.core.points)
        return (
            self._nav_client.server_is_ready()
            and (not needs_spray or (
                self._spray_client.server_is_ready()
                and self._motion_control_state == 'RUNNING'))
            and (not self._require_relay_service or self._relay.service_is_ready()))

    def _clear_nav_startup_retry(self):
        self._initial_nav_started = None
        self._nav_retry_due = None

    def _begin_mission_navigation(self):
        self.core.start()
        self._initial_nav_started = self._now()
        self._nav_retry_due = None
        self._start_navigation()

    def _schedule_initial_nav_retry(self):
        """
        处理 Nav2 启动时的重试。
        
        原因：Nav2 的 Action 服务可能在 Lifecycle (生命周期) 节点完全激活前
        就已经发布。为了避免首次目标被误判为“拒绝”，在 30 秒的窗口期内
        提供了一次性重试机制。
        """
        now = self._now()
        started = self._initial_nav_started
        if (not self._navigation_active() or self.core.current_index != 0 or
                started is None or
                now - started > self._nav_startup_retry_timeout):
            return False
        self._nav_retry_due = now + self._nav_startup_retry_interval
        self.get_logger().warn(
            '[NAV] initial goal rejected while Nav2 is activating; '
            f'retrying in {self._nav_startup_retry_interval:.1f}s')
        self._publish_status()
        return True

    def _send_nav_goal(self):
        """发送 Nav2 导航目标并启动超时计时。"""
        if not self._navigation_active():
            return
        if self.core.state == MissionState.RETURNING_HOME:
            x, y, yaw = self._home_pose
            target_label = 'HOME'
        else:
            point = self.core.current_point
            if point is None:
                self._fail('no current navigation point')
                return
            x, y, yaw = point.docking_pose
            target_label = point.point_id
        goal = NavigateToPose.Goal()
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.header.frame_id = self._map_frame
        self._set_pose(goal.pose.pose, x, y, yaw)
        self._nav_pending = True
        self._phase_started = self._now()
        future = self._nav_client.send_goal_async(goal)
        future.add_done_callback(self._nav_goal_response)
        self.get_logger().info(
            f'[NAV] sent point={target_label} '
            f'pose=({x:.2f},{y:.2f},{yaw:.2f})')
        self._publish_status()

    def _start_navigation(self):
        """Ensure wide spray is off for a disabled incoming segment first."""
        self._reset_wide_spray_motion()
        if self.core.state == MissionState.RETURNING_HOME:
            self._relay.command(
                self._wide_relay_channel, False, 0.0,
                self._send_nav_goal, 'returning home: disable wide spray')
            return
        point = self.core.current_point
        if point is None:
            self._fail('no current navigation point')
            return
        if point.wide_spray_on_approach:
            self._send_nav_goal()
            return
        self._relay.command(
            self._wide_relay_channel, False, 0.0,
            self._send_nav_goal,
            f'{point.point_id}: approach has wide spray disabled')

    def _navigation_active(self):
        return self.core.state in (
            MissionState.NAVIGATING, MissionState.RETURNING_HOME)

    def _begin_stop_verification(self):
        if self.core.state != MissionState.VERIFYING_STOP:
            return
        point = self.core.current_point
        if point is None:
            self._fail('cannot verify stop without a route point')
            return

        # A zero transit duration means that Nav2 arrival and the confirmed
        # channel-1 OFF command are sufficient for a non-arm route point.
        # INSPECT points never enter this path: arm motion still requires the
        # normal fresh-odometry stop confirmation.
        if (not point.requires_arm and
                self._transit_stop_stable_duration == 0.0):
            self._phase_started = self._now()
            if not self.core.stop_verified():
                self._fail('cannot skip transit stop verification')
                return
            self.get_logger().info(
                '[STOP_CHECK] transit confirmation skipped (0.0s)')
            if point.dwell_time_sec <= 0.0:
                self._finish_noninspect_point()
            else:
                self._publish_status()
            return

        self._phase_started = self._now()
        stable_duration = (
            self._stop_stable_duration if point.requires_arm
            else self._transit_stop_stable_duration)
        self._stop_detector.start(self._phase_started, stable_duration)
        self._publish_status()

    def _skip_navigation_point(self, reason):
        """Continue after a Nav2 result that explicitly ended the goal."""
        self._reset_wide_spray_motion()

        def skip_after_wide_off():
            point = self.core.current_point
            if point is None or not self.core.skip_current(
                    self._return_home_after_mission, reason):
                self._fail(reason)
                return
            self._stop_detector.stop()
            self.get_logger().warning(
                f'[MISSION] skipped point={point.point_id}: {reason}')
            self._continue_after_point()

        # A failed navigation can occur with channel 1 active.  The off
        # command must be issued before this route point is discarded.
        self._relay.command(
            self._wide_relay_channel, False, 0.0, skip_after_wide_off,
            f'navigation failure: disable wide spray before skip ({reason})')

    def _finish_noninspect_point(self):
        point = self.core.current_point
        if point is None or not self.core.point_succeeded(
                self._return_home_after_mission, 'route point completed'):
            self._fail('cannot complete non-inspect route point')
            return
        self.get_logger().info(f'[MISSION] completed point={point.point_id}')
        self._continue_after_point()

    def _continue_after_point(self):
        if self.core.state in {
                MissionState.NAVIGATING, MissionState.RETURNING_HOME}:
            self._start_navigation()
            return
        self._command_all_relays_off()
        if self.core.state == MissionState.MISSION_COMPLETED:
            terminal = (
                'MISSION_COMPLETED' if self.core.all_points_completed
                else 'MISSION_FINISHED_INCOMPLETE')
            self.get_logger().info(
                f'[MISSION] {terminal} total={len(self.core.points)} '
                f'completed={self.core.completed_points} '
                f'partial={self.core.partial_points} '
                f'skipped={self.core.skipped_points}')
        self._publish_status()

    def _relay_required_failure(self, detail):
        """Fail closed without recursively issuing another relay request.

        A failed ON command means the visible treatment was not delivered; a
        failed OFF command leaves the physical output state unknown.  In both
        cases the only safe route action is to stop the vehicle, cancel active
        work and latch the mission in FAILED.  We deliberately do not call
        ``_command_all_relays_off`` here because the service itself is the
        failing dependency.
        """
        self._reset_wide_spray_motion()
        self._stop_detector.stop()
        self._clear_nav_startup_retry()
        self._cancel_nav_goal()
        self._cancel_spray_goal()
        self._publish_motion_command('stop')
        message = f'required relay command failed: {detail}'
        self.core.fail(message)
        self.get_logger().error(f'[MISSION][RELAY] {message}')
        self._publish_status()

    def _command_all_relays_off(self):
        self._reset_wide_spray_motion()
        self._relay.command_all_off()

    def _reset_wide_spray_motion(self):
        self._wide_motion_pending = False
        self._wide_motion_deadline = None
        self._wide_stop_started_at = None
        self._wide_stop_off_pending = False

    def _tick_wide_spray_motion(self, now):
        if self.core.state != MissionState.NAVIGATING:
            self._reset_wide_spray_motion()
            return
        point = self.core.current_point
        if point is None or not point.wide_spray_on_approach:
            self._reset_wide_spray_motion()
            return

        odom_is_fresh = (
            self._last_odom_at is not None and
            now - self._last_odom_at <= self._odom_stale_timeout)
        if self._wide_motion_pending:
            if (self._wide_motion_deadline is not None and
                    now >= self._wide_motion_deadline):
                self._wide_motion_pending = False
                self._wide_motion_deadline = None
                self.get_logger().warning(
                    '[MISSION][WARN][RELAY] vehicle did not start moving before '
                    'wide spray timeout; continuing with channel 1 off')
            elif (odom_is_fresh and
                  self._latest_linear_speed >= self._wide_motion_linear_threshold):
                self._wide_motion_pending = False
                self._wide_motion_deadline = None
                self._relay.command(
                    self._wide_relay_channel, True, 0.0, None,
                    f'{point.point_id}: vehicle motion confirmed; enable wide spray')

        if self._wide_motion_pending or self._wide_stop_off_pending:
            return
        if not self._relay.wide_enabled:
            self._wide_stop_started_at = None
            return
        if not odom_is_fresh:
            self._disable_wide_spray_after_stop(
                point, 'odometry became stale; disable wide spray')
            return
        if (self._latest_linear_speed >= self._linear_stop_threshold or
                self._latest_angular_speed >= self._angular_stop_threshold):
            self._wide_stop_started_at = None
            return
        if self._wide_stop_started_at is None:
            self._wide_stop_started_at = now
            return
        if now - self._wide_stop_started_at >= self._stop_stable_duration:
            self._disable_wide_spray_after_stop(
                point, 'vehicle remained stopped; disable wide spray')

    def _disable_wide_spray_after_stop(self, point, reason):
        """Turn channel 1 off once, then wait for this segment to move again."""
        if self._wide_stop_off_pending or not self._relay.wide_enabled:
            return
        point_id = point.point_id
        self._wide_stop_off_pending = True
        self._wide_motion_pending = False
        self._wide_motion_deadline = None

        def rearm_after_confirmed_off():
            self._wide_stop_off_pending = False
            self._wide_stop_started_at = None
            current = self.core.current_point
            if (self.core.state == MissionState.NAVIGATING and
                    current is not None and
                    current.point_id == point_id and
                    current.wide_spray_on_approach):
                self._wide_motion_pending = True
                # A recovery can last longer than the initial 10-second
                # startup gate. Re-enable only after fresh forward motion.
                self._wide_motion_deadline = None

        self._relay.command(
            self._wide_relay_channel, False, 0.0, rearm_after_confirmed_off,
            f'{point_id}: {reason}')

    def _publish_motion_command(self, command):
        self._motion_command_pub.publish(String(data=str(command)))

    def _advance_abort_and_home(self):
        if not self._abort_and_home_requested or self._abort_reset_sent:
            return
        if any((self._nav_pending, self._spray_pending,
                self._nav_handle is not None, self._spray_handle is not None)):
            return
        self._abort_reset_sent = True
        self._abort_reset_started = False
        self._publish_motion_command('reset')
        self.core.last_error = 'abort_and_home: motion_control reset requested'
        self._publish_status()

    def _pause_for_recovery(self, message):
        returning_home = self.core.state == MissionState.RETURNING_HOME
        if not self.core.pause_for_recovery():
            self._fail(message)
            return
        self._recovery_return_home = returning_home
        self._stop_detector.stop()
        self._clear_nav_startup_retry()
        self._cancel_nav_goal()
        self._command_all_relays_off()
        self.core.last_error = str(message)
        self._publish_status()

    def _nav_goal_response(self, future):
        self._nav_pending = False
        try:
            handle = future.result()
        except Exception as error:
            self._nav_timeout_canceling = False
            self._nav_timeout_cancel_deadline = None
            if self._navigation_active():
                if self.core.state == MissionState.RETURNING_HOME:
                    self._pause_for_recovery(f'Nav2 HOME goal send failed: {error}')
                else:
                    self._skip_navigation_point(f'Nav2 goal send failed: {error}')
            self._advance_abort_and_home()
            return
        if handle is None or not handle.accepted:
            self._nav_timeout_canceling = False
            self._nav_timeout_cancel_deadline = None
            if self._schedule_initial_nav_retry():
                return
            if self._navigation_active():
                if self.core.state == MissionState.RETURNING_HOME:
                    self._pause_for_recovery('Nav2 rejected the HOME goal')
                else:
                    self._skip_navigation_point('Nav2 rejected the goal')
            self._advance_abort_and_home()
            return
        self._nav_handle = handle
        self._clear_nav_startup_retry()
        if self._nav_timeout_canceling or not self._navigation_active():
            self._cancel_nav_goal()
        elif (self.core.state == MissionState.NAVIGATING and
              self.core.current_point is not None and
              self.core.current_point.wide_spray_on_approach):
            self._wide_motion_pending = True
            self._wide_motion_deadline = (
                self._now() + self._wide_motion_timeout)
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._nav_result)

    def _nav_result(self, future):
        self._nav_handle = None
        self._nav_timeout_canceling = False
        self._nav_timeout_cancel_deadline = None
        try:
            wrapped = future.result()
        except Exception as error:
            if self._navigation_active():
                if self.core.state == MissionState.RETURNING_HOME:
                    self._pause_for_recovery(f'Nav2 HOME result failed: {error}')
                else:
                    self._skip_navigation_point(f'Nav2 result failed: {error}')
            self._advance_abort_and_home()
            return
        if not self._navigation_active():
            self._publish_status()
            self._advance_abort_and_home()
            return
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            if self.core.state == MissionState.RETURNING_HOME:
                self._pause_for_recovery(
                    f'Nav2 HOME failed with status {wrapped.status}')
                return
            result = getattr(wrapped, 'result', None)
            error_code = getattr(result, 'error_code', None)
            error_message = str(getattr(result, 'error_msg', '')).strip()
            details = []
            if error_code is not None:
                details.append(f'error_code={error_code}')
            if error_message:
                details.append(error_message)
            suffix = f" ({'; '.join(details)})" if details else ''
            self._skip_navigation_point(
                f'Nav2 failed with status {wrapped.status}{suffix}')
            return
        if self.core.state == MissionState.RETURNING_HOME:
            self.core.home_succeeded(canceled=self._manual_return_home)
            self.get_logger().info(
                f'[NAV] HOME reached; state={self.core.state.name}')
            self._publish_status()
            return
        self._navigation_arrived()

    def _navigation_arrived(self):
        """Enter the existing relay/stop gate after final pose convergence."""
        self._reset_wide_spray_motion()
        if self.core.state == MissionState.NAVIGATING:
            if not self.core.nav_succeeded():
                self._fail('cannot enter stop verification after navigation result')
                return
        elif self.core.state != MissionState.VERIFYING_STOP:
            self._fail('cannot enter stop verification from current mission state')
            return
        point = self.core.current_point
        if point is None:
            self._fail('navigation completed without a route point')
            return
        self.get_logger().info(f'[NAV] succeeded point={point.point_id}')
        self._relay.command(
            self._wide_relay_channel, False, 0.0,
            self._begin_stop_verification,
            f'{point.point_id}: disable wide spray at stop')

    def _on_odom(self, message):
        linear = math.hypot(message.twist.twist.linear.x, message.twist.twist.linear.y)
        angular = abs(message.twist.twist.angular.z)
        now = self._now()
        self._last_odom_at = now
        self._latest_linear_speed = linear
        self._latest_angular_speed = angular
        self._stop_detector.update(now, linear, angular)

    def _on_motion_control_state(self, message):
        self._motion_control_state = str(message.data)
        if not (self._abort_and_home_requested and self._abort_reset_sent):
            return
        if self._motion_control_state == 'RESETTING':
            self._abort_reset_started = True
        elif (self._motion_control_state == 'RUNNING' and
              self._abort_reset_started):
            self._abort_and_home_requested = False
            self._abort_reset_sent = False
            self._abort_reset_started = False
            self.core.last_error = 'abort_and_home: arm HOME complete and ready'
            self._publish_status()
        elif self._motion_control_state == 'RESET_FAILED':
            self.core.last_error = 'abort_and_home: arm HOME reset failed'
            self._publish_status()

    def _send_spray_goal(self):
        """构建并发送机械臂 Action 目标。"""
        point = self.core.current_point
        goal = ExecuteSpray.Goal()
        goal.mission_id = self.core.mission_id
        goal.spray_duration = point.spray_duration
        goal.tree_hint.header.stamp = self.get_clock().now().to_msg()
        goal.tree_hint.header.frame_id = self._map_frame
        goal.tree_hint.point.x = point.x
        goal.tree_hint.point.y = point.y
        goal.tree_hint.point.z = point.z
        # Full missions keep the calibrated dynamic tree-plane calculation.
        # A positive override is reserved for the standalone arm-test UI.
        goal.working_range_m = 0.0
        self._spray_pending = True
        self._phase_started = self._now()
        self._spray_last_progress = self._phase_started
        future = self._spray_client.send_goal_async(
            goal, feedback_callback=self._spray_feedback)
        future.add_done_callback(self._spray_goal_response)
        self._publish_status()

    def _skip_arm_point(self, reason):
        point = self.core.current_point
        if point is None or not self.core.skip_current(
                self._return_home_after_mission, reason):
            self._fail(reason)
            return
        self._manual_return_home = False
        self.get_logger().warning(
            f'[MISSION] skipped arm point={point.point_id}: {reason}')
        self._continue_after_point()

    def _spray_goal_response(self, future):
        self._spray_pending = False
        try:
            handle = future.result()
        except Exception as error:
            if self.core.state == MissionState.ARM_SPRAYING:
                self._fail(f'spray goal send failed: {error}')
            self._advance_abort_and_home()
            return
        if handle is None or not handle.accepted:
            if self.core.state == MissionState.ARM_SPRAYING:
                self._skip_arm_point('spray Action rejected the goal')
            self._advance_abort_and_home()
            return
        self._spray_handle = handle
        if self.core.state != MissionState.ARM_SPRAYING:
            self._cancel_spray_goal()
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._spray_result)

    def _spray_feedback(self, feedback_message):
        feedback = feedback_message.feedback
        if self.core.state == MissionState.ARM_SPRAYING:
            self._spray_last_progress = self._now()
        self.get_logger().debug(
            f'[ARM] {feedback.phase_text} progress={feedback.progress:.2f}')

    def _spray_result(self, future):
        self._spray_handle = None
        self._spray_last_progress = None
        try:
            wrapped = future.result()
        except Exception as error:
            if self.core.state == MissionState.ARM_SPRAYING:
                self._fail(f'spray result failed: {error}')
            self._advance_abort_and_home()
            return
        if self.core.state != MissionState.ARM_SPRAYING:
            self._publish_status()
            self._advance_abort_and_home()
            return
        result = wrapped.result
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED or not result.success:
            reason = (
                f'spray failed status={wrapped.status} '
                f'code={result.error_code}: {result.message}')
            if result.error_code in {
                    ExecuteSpray.Result.CANCELED,
                    ExecuteSpray.Result.HOME_FAILED,
                    ExecuteSpray.Result.LOCKED,
                    ExecuteSpray.Result.INTERNAL_ERROR}:
                self._fail(reason)
            else:
                self._skip_arm_point(reason)
            return
        finished = self.core.current_point.point_id
        self._manual_return_home = False
        if result.error_code == ExecuteSpray.Result.INSPECTED_NO_DISEASE:
            outcome = 'inspected without disease'
            self.core.arm_succeeded(
                self._return_home_after_mission, result.message)
        elif result.error_code == ExecuteSpray.Result.PARTIAL_SUCCESS:
            outcome = 'partially sprayed'
            self.core.arm_partial(
                result.message, self._return_home_after_mission)
            self.get_logger().warn(
                f'[MISSION] partial point={finished}: {result.message}')
        else:
            outcome = 'sprayed'
            self.core.arm_succeeded(
                self._return_home_after_mission, result.message)
        self.get_logger().info(
            f'[MISSION] {outcome} point={finished} '
            f'processed={self.core.processed_points}/'
            f'{len(self.core.points)} completed={self.core.completed_points} '
            f'partial={self.core.partial_points} '
            f'skipped={self.core.skipped_points}: {result.message}')
        if self.core.state in {
                MissionState.NAVIGATING, MissionState.RETURNING_HOME}:
            self._start_navigation()
        else:
            if self.core.state == MissionState.MISSION_COMPLETED:
                self.get_logger().info(
                    f'[MISSION] MISSION_COMPLETED points={len(self.core.points)} '
                    f'completed={self.core.completed_points} '
                    f'partial={self.core.partial_points} '
                    f'skipped={self.core.skipped_points}')
            self._continue_after_point()

    # ---------- 100ms 调度看门狗 (Tick) ----------
    def _tick(self):
        """Advance watchdogs in the existing abort, navigation, then work order."""
        now = self._now()
        self._advance_abort_and_home()
        if self._abort_and_home_requested:
            return
        self._tick_wide_spray_motion(now)
        if self._tick_startup_retry(now):
            return
        if self._tick_navigation_timeout(now):
            return
        self._tick_active_work_phase(now)

    def _tick_startup_retry(self, now):
        """Run, and consume this tick for, a pending first-Nav2-goal retry."""
        if self._nav_retry_due is None:
            return False
        if not self._navigation_active():
            self._clear_nav_startup_retry()
        elif now >= self._nav_retry_due:
            self._nav_retry_due = None
            self._start_navigation()
        return True

    def _tick_navigation_timeout(self, now):
        """Cancel a timed-out Nav2 goal before applying the existing recovery path."""
        if self._navigation_active() and self._phase_started is not None:
            if now - self._phase_started >= self._nav_timeout:
                if not self._nav_timeout_canceling:
                    self._nav_timeout_canceling = True
                    self._nav_timeout_cancel_deadline = now + 5.0
                    self._phase_started = None
                    self._cancel_nav_goal()
                    self.get_logger().warning(
                        '[NAV] goal timed out; canceling before skipping point')
        if not self._nav_timeout_canceling:
            return False
        if (self._nav_timeout_cancel_deadline is not None and
                now >= self._nav_timeout_cancel_deadline):
            self._nav_timeout_canceling = False
            self._nav_timeout_cancel_deadline = None
            self._pause_for_recovery(
                'Nav2 timeout cancellation did not settle; relocalize and resume')
        return True

    def _tick_active_work_phase(self, now):
        """Advance stop verification, transit dwell, or arm-action watchdogs."""
        if self.core.state == MissionState.VERIFYING_STOP:
            status = self._stop_detector.status(now)
            if status == StopDetector.STABLE:
                point = self.core.current_point
                if point is None:
                    self._fail('stop verified without a route point')
                else:
                    self._stop_detector.stop()
                    self.core.stop_verified()
                    self.get_logger().info(
                        '[STOP_CHECK] vehicle is stable')
                    if self.core.state == MissionState.ARM_SPRAYING:
                        self._send_spray_goal()
                    elif self.core.state == MissionState.DWELLING:
                        self._phase_started = now
                        if point.dwell_time_sec <= 0.0:
                            self._finish_noninspect_point()
                        else:
                            self._publish_status()
            elif status in (StopDetector.STALE, StopDetector.TIMEOUT):
                point = self.core.current_point
                if point is not None and point.requires_arm:
                    # Nav2 has already reported arrival.  Without a verified
                    # stationary vehicle the arm must not move, but this one
                    # inspection point must not terminate the remaining route.
                    self._skip_arm_point(
                        f'odom stop verification failed: {status}')
                else:
                    self._skip_navigation_point(
                        f'odom stop verification failed: {status}')
        elif self.core.state == MissionState.DWELLING:
            point = self.core.current_point
            if point is None:
                self._fail('dwell without a route point')
            elif (self._phase_started is not None and
                  now - self._phase_started >= point.dwell_time_sec):
                self._finish_noninspect_point()
        elif self.core.state == MissionState.ARM_SPRAYING and self._phase_started is not None:
            if now - self._phase_started >= self._spray_timeout:
                self._fail('spray Action timed out')
            elif (self._spray_last_progress is not None and
                  now - self._spray_last_progress >= self._spray_progress_timeout):
                self._fail('spray Action made no progress')

    def _fail(self, message):
        if not self.core.fail(message):
            return
        self._clear_nav_startup_retry()
        self._stop_detector.stop()
        self._cancel_nav_goal()
        self._cancel_spray_goal()
        self._command_all_relays_off()
        self.get_logger().error(f'[MISSION] FAILED: {message}')
        self._publish_status()

    def _cancel_nav_goal(self):
        if self._nav_handle is not None:
            self._nav_handle.cancel_goal_async()

    def _cancel_spray_goal(self):
        if self._spray_handle is not None:
            self._spray_handle.cancel_goal_async()

    def _publish_status(self):
        """向 Qt/状态机发布 Transient Local 任务快照。"""
        message = MissionStatus()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._map_frame
        message.mission_id = self.core.mission_id
        message.state = int(self.core.state)
        message.state_text = (
            'MISSION_FINISHED_INCOMPLETE'
            if (self.core.state == MissionState.MISSION_COMPLETED and
                not self.core.all_points_completed)
            else self.core.state.name)
        point = self.core.current_point
        message.current_point_id = point.point_id if point else ''
        message.current_index = self.core.current_index
        message.total_points = len(self.core.points)
        message.completed_points = self.core.completed_points
        message.skipped_points = self.core.skipped_points
        for index, point_item in enumerate(self.core.points):
            point_status = MissionPointStatus()
            point_status.point_id = point_item.point_id
            outcome = self.core.point_outcomes[index]
            if outcome == MissionCore.COMPLETED:
                point_status.state = MissionPointStatus.COMPLETED
            elif outcome == MissionCore.SKIPPED:
                point_status.state = MissionPointStatus.SKIPPED
            elif outcome in {MissionCore.PARTIAL, MissionCore.FAILED}:
                point_status.state = MissionPointStatus.FAILED
                point_status.message = self.core.point_messages[index]
            elif index == self.core.current_index:
                point_status.state = MissionPointStatus.CURRENT
            else:
                point_status.state = MissionPointStatus.PENDING
            point_status.state_text = (
                'CURRENT' if point_status.state == MissionPointStatus.CURRENT
                else outcome)
            message.point_statuses.append(point_status)
        message.last_error = self.core.last_error
        message.nav_goal_active = self._nav_pending or self._nav_handle is not None
        message.arm_goal_active = self._spray_pending or self._spray_handle is not None
        self._status_pub.publish(message)

    @staticmethod
    def _set_pose(pose, x, y, yaw):
        pose.position.x = x
        pose.position.y = y
        pose.orientation.z = math.sin(yaw / 2.0)
        pose.orientation.w = math.cos(yaw / 2.0)

    def _publish_plan(self):
        message = MissionPlan()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._map_frame
        message.mission_id = self.core.mission_id
        message.return_home_after_mission = self._return_home_after_mission
        self._set_pose(message.home_pose, *self._home_pose)
        for point in self.core.points:
            item = MissionPointPlan()
            item.point_id = point.point_id
            item.point_type = int(point.point_type)
            item.tree_hint.x = point.x
            item.tree_hint.y = point.y
            item.tree_hint.z = point.z
            item.spray_duration = point.spray_duration
            item.tree_x_m = point.tree_x_m
            item.tree_y_m = point.tree_y_m
            self._set_pose(item.docking_pose, *point.docking_pose)
            message.points.append(item)
        self._plan_pub.publish(message)


def main():
    rclpy.init()
    node = MissionManager()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
