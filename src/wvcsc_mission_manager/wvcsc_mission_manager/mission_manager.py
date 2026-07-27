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
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from wvcsc_interfaces.action import ExecuteSpray
from wvcsc_interfaces.msg import (
    MissionPlan,
    MissionStatus,
    MissionTargetPlan,
    MissionTargetStatus,
)
from wvcsc_interfaces.srv import LoadManualMission, SetRelay

from .core import (
    arm_base_xy,
    DEFAULT_ARM_BASE_FORWARD_OFFSET,
    DEFAULT_ARM_BASE_LEFT_OFFSET,
    MissionCore,
    MissionState,
    PointType,
    StopDetector,
    Target,
    WorkSide,
    tree_hint_from_arm_base_offset,
)


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
        self._require_docking_quality = bool(
            self.get_parameter('require_docking_quality').value)
        self._docking_pose_source = str(
            self.get_parameter('docking_pose_source').value).strip().lower()
        self._accept_aborted_near_goal = bool(
            self.get_parameter('accept_aborted_near_goal').value)
        self._localization_max_age = float(
            self.get_parameter('localization_max_age_sec').value)
        self._max_localization_position_stddev = float(
            self.get_parameter('max_localization_position_stddev_m').value)
        self._max_localization_yaw_stddev = float(
            self.get_parameter('max_localization_yaw_stddev_rad').value)
        self._nav_goal_xy_tolerance = float(
            self.get_parameter('nav_goal_xy_tolerance_m').value)
        self._nav_goal_yaw_tolerance = float(
            self.get_parameter('nav_goal_yaw_tolerance_rad').value)
        self._inspect_nav_behavior_tree = str(
            self.get_parameter('inspect_nav_behavior_tree').value).strip()
        self._route_nav_behavior_tree = str(
            self.get_parameter('route_nav_behavior_tree').value).strip()
        self._max_docking_position_error = float(
            self.get_parameter('max_docking_position_error_m').value)
        self._max_docking_yaw_error = float(
            self.get_parameter('max_docking_yaw_error_rad').value)
        self._docking_retry_limit = int(
            self.get_parameter('docking_retry_limit').value)
        self._localization_recovery_timeout = float(
            self.get_parameter('localization_recovery_timeout_sec').value)
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
        self._return_home_after_finish = bool(
            self.get_parameter('return_home_after_finish').value)
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
                self._wide_motion_timeout)):
            raise ValueError('arm base and wide spray parameters must be finite')
        if self._wide_motion_linear_threshold < 0.0 or self._wide_motion_timeout <= 0.0:
            raise ValueError('wide spray motion thresholds are invalid')
        if (not math.isfinite(self._nav_startup_retry_timeout) or
                not math.isfinite(self._nav_startup_retry_interval) or
                self._nav_startup_retry_timeout <= 0.0 or
                self._nav_startup_retry_interval <= 0.0):
            raise ValueError('initial Nav2 retry timing must be finite and positive')
        docking_thresholds = (
            self._localization_max_age,
            self._max_localization_position_stddev,
            self._max_localization_yaw_stddev,
            self._nav_goal_xy_tolerance,
            self._nav_goal_yaw_tolerance,
            self._max_docking_position_error,
            self._max_docking_yaw_error,
            self._localization_recovery_timeout,
        )
        if not all(math.isfinite(value) and value > 0.0
                   for value in docking_thresholds):
            raise ValueError(
                'docking quality thresholds must be finite and positive')
        if self._docking_retry_limit < 0:
            raise ValueError('docking_retry_limit must be non-negative')
        if self._docking_pose_source not in {'localization', 'odom'}:
            raise ValueError(
                'docking_pose_source must be localization or odom')
        self._configured_return_home_after_finish = (
            self._return_home_after_finish)
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
        self._wide_active_pub = self.create_publisher(
            Bool, '/spray/wide_active', latched)

        # 订阅里程计和定位质量；任务仅通过 /mission/load_manual 注入。
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.create_subscription(
            PoseWithCovarianceStamped,
            str(self.get_parameter('localization_pose_topic').value),
            self._on_localization_pose, 10)
        # 初始化 Action 客户端
        self._nav_client = ActionClient(
            self, NavigateToPose, str(self.get_parameter('nav_action_name').value))
        self._spray_client = ActionClient(
            self, ExecuteSpray, str(self.get_parameter('spray_action_name').value))
        self._relay_client = self.create_client(
            SetRelay, str(self.get_parameter('relay_service_name').value))
        self._motion_command_pub = self.create_publisher(
            String, '/motion_control/command', 10)

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
        self._localization_pose = None
        self._odom_docking_pose = None
        self._localization_recovery_started = None
        self._docking_retry_count = 0
        self._docking_retry_target_index = None
        self._last_docking_log_state = None
        self._last_odom_at = None
        self._latest_linear_speed = 0.0
        self._wide_motion_pending = False
        self._wide_motion_deadline = None
        self._wide_relay_enabled = False
        self._publish_wide_active()
        self._relay_failure_latched = False
        self._abort_and_home_requested = False
        self._abort_reset_sent = False
        self._motion_control_state = ''
        self._recovery_return_home = False
        self._recovery_pause = False
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
            String, '/motion_control/state', self._on_motion_control_state, 10)

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
            'require_docking_quality': False,
            'docking_pose_source': 'localization',
            'accept_aborted_near_goal': False,
            'localization_pose_topic': '/amcl_pose',
            'localization_max_age_sec': 1.0,
            'max_localization_position_stddev_m': 0.12,
            'max_localization_yaw_stddev_rad': 0.12,
            'nav_goal_xy_tolerance_m': 0.10,
            'nav_goal_yaw_tolerance_rad': 0.10,
            # Nav2 supports a per-goal behavior tree.  Inspection points use
            # the strict checker required by the arm docking gate; transit and
            # finish points use a relaxed tree so they do not hunt an arm pose.
            'inspect_nav_behavior_tree': '',
            'route_nav_behavior_tree': '',
            'max_docking_position_error_m': 0.12,
            'max_docking_yaw_error_rad': 0.12,
            'docking_retry_limit': 1,
            'localization_recovery_timeout_sec': 5.0,
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
            'return_home_after_finish': False,
            'home_x': 0.0,
            'home_y': 0.0,
            'home_yaw': 0.0,
            'linear_stop_threshold': 0.03,
            'angular_stop_threshold': 0.03,
            'stop_stable_duration_sec': 1.0,
            'odom_stale_timeout_sec': 1.0,
            'stop_verify_timeout_sec': 5.0,
            # Qt 实车路线可包含 23 株病株以及通行/终点，不能沿用旧的
            # 二十目标上限；仍保留有限上界防止错误任务文件无限膨胀。
            'max_targets': 64,
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
        try:
            targets, home_pose = self._validate_manual_request(request)
            outcome = self.core.load(request.mission_id.strip(), targets)
        except ValueError as error:
            self.get_logger().error(f'[MISSION] rejected manual task list: {error}')
            return self._reply(response, False, str(error))
        if outcome != 'accepted':
            message = (
                'duplicate mission id' if outcome == 'duplicate'
                else 'mission manager is busy')
            return self._reply(response, False, message)
        self._home_pose = home_pose
        self._return_home_after_finish = bool(
            request.return_home_after_finish)
        self._manual_return_home = False
        self._abort_and_home_requested = False
        self._abort_reset_sent = False
        self._recovery_return_home = False
        self._recovery_pause = False
        self._nav_timeout_canceling = False
        self._nav_timeout_cancel_deadline = None
        self._clear_nav_startup_retry()
        self._reset_docking_verification()
        self.get_logger().info(
            f'[MISSION] accepted manual mission={self.core.mission_id} '
            f'targets={len(self.core.targets)}')
        self._publish_plan()
        self._publish_status()
        return self._reply(response, True, 'manual mission loaded')

    def _validate_manual_request(self, request):
        """Validate the single Qt/RViz task contract before loading it.

        Every point supplies an explicit parking pose.  Inspect points also
        supply a signed tree-root offset in Alicia base coordinates; the map
        tree hint is derived once here and never accepted from another source.
        """
        if request.header.frame_id != self._map_frame:
            raise ValueError(f'frame must be {self._map_frame}')
        if not request.mission_id.strip() or not request.targets:
            raise ValueError('mission_id and targets are required')
        max_targets = int(self.get_parameter('max_targets').value)
        if len(request.targets) > max_targets:
            raise ValueError(f'target count exceeds limit {max_targets}')

        bound = float(self.get_parameter('max_abs_coordinate').value)
        min_duration = float(self.get_parameter('min_spray_duration').value)
        max_duration = float(self.get_parameter('max_spray_duration').value)
        home_pose = MissionManager._pose_to_xy_yaw(request.home_pose, 'home_pose')
        if abs(home_pose[0]) > bound or abs(home_pose[1]) > bound:
            raise ValueError('home_pose is out of bounds')

        seen = set()
        targets = []
        for item in request.targets:
            target_id = item.target_id.strip()
            if not target_id or target_id in seen:
                raise ValueError('target_id must be non-empty and unique')
            point_type = int(getattr(item, 'point_type', PointType.INSPECT))
            if point_type not in set(PointType):
                raise ValueError(f'{target_id}: unsupported point_type')
            work_side = int(getattr(item, 'work_side', WorkSide.UNSPECIFIED))
            if work_side not in set(WorkSide):
                raise ValueError(f'{target_id}: unsupported work_side')
            docking = MissionManager._pose_to_xy_yaw(
                item.docking_pose, f'{target_id}.docking_pose')
            if abs(docking[0]) > bound or abs(docking[1]) > bound:
                raise ValueError(f'{target_id}: docking pose out of bounds')
            dwell_time = float(getattr(item, 'dwell_time_sec', 0.0))
            if not math.isfinite(dwell_time) or dwell_time < 0.0:
                raise ValueError(f'{target_id}: dwell_time_sec must be non-negative')
            wide_spray_on_approach = bool(getattr(
                item, 'wide_spray_on_approach', False))
            if point_type != PointType.INSPECT:
                targets.append(Target(
                    target_id, 0.0, 0.0, 0.0, 0.0, docking,
                    point_type=point_type,
                    wide_spray_on_approach=wide_spray_on_approach,
                    dwell_time_sec=dwell_time,
                    work_side=work_side))
                seen.add(target_id)
                continue

            configured_duration = float(getattr(
                item, 'arm_spray_duration_sec', 0.0))
            duration = configured_duration if configured_duration > 0.0 else float(
                item.spray_duration)
            if (not math.isfinite(duration) or
                    not min_duration <= duration <= max_duration):
                raise ValueError(f'{target_id}: spray_duration out of range')
            tree_x_m = float(item.tree_x_m)
            tree_y_m = float(item.tree_y_m)
            tree_base_z = float(item.tree_base_z_m)
            if not all(math.isfinite(value) for value in (
                    tree_x_m, tree_y_m, tree_base_z)):
                raise ValueError(f'{target_id}: non-finite arm-base tree offset')
            if math.hypot(tree_x_m, tree_y_m) < 1e-6:
                raise ValueError(f'{target_id}: arm-base tree offset is zero')
            if work_side != WorkSide.UNSPECIFIED:
                expected_side = WorkSide.LEFT if tree_y_m > 0.0 else WorkSide.RIGHT
                if abs(tree_y_m) <= 0.05 or work_side != expected_side:
                    raise ValueError(
                        f'{target_id}: work_side conflicts with signed tree Y')
            tree_hint = tree_hint_from_arm_base_offset(
                docking, tree_x_m, tree_y_m, tree_base_z,
                self._arm_base_forward_offset,
                self._arm_base_left_offset,
                self._arm_base_yaw)
            if abs(tree_hint[0]) > bound or abs(tree_hint[1]) > bound:
                raise ValueError(f'{target_id}: derived tree hint out of bounds')
            targets.append(Target(
                target_id, tree_hint[0], tree_hint[1], tree_hint[2],
                duration, docking, tree_x_m, tree_y_m,
                point_type, wide_spray_on_approach, dwell_time, work_side))
            seen.add(target_id)
        return targets, home_pose

    @staticmethod
    def _pose_to_xy_yaw(pose, label):
        values = (
            pose.position.x, pose.position.y, pose.position.z,
            pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w)
        if not all(math.isfinite(value) for value in values):
            raise ValueError(f'{label}: non-finite pose')
        norm = math.sqrt(sum(value * value for value in values[3:]))
        if norm < 1e-6:
            raise ValueError(f'{label}: invalid orientation')
        x, y, z, w = (value / norm for value in values[3:])
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        return pose.position.x, pose.position.y, yaw

    def _restore_configured_mission_options(self):
        self._home_pose = self._configured_home_pose
        self._return_home_after_finish = (
            self._configured_return_home_after_finish)

    def _reset_docking_verification(self, *, reset_retry=True):
        self._localization_recovery_started = None
        self._last_docking_log_state = None
        if reset_retry:
            self._docking_retry_count = 0
            self._docking_retry_target_index = None

    # ---------- Service Callbacks (开始、暂停、取消、跳过、复位) ----------
    def _start(self, _request, response):
        if self.core.state != MissionState.READY:
            return self._reply(response, False, 'mission is not READY')
        if not self._servers_ready():
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
        if (self._recovery_pause and
                not self._localization_ready_for_resume()):
            return self._reply(
                response, False,
                'fresh, confident AMCL localization is required before resume')
        self.core.resume(returning_home=self._recovery_return_home)
        self._recovery_return_home = False
        self._recovery_pause = False
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
        """Cancel the route, then request a locked MoveIt HOME reset.

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
        target = self.core.current_target
        if target is None or not self.core.skip_current(
                self._return_home_after_finish):
            return self._reply(
                response, False,
                'skip is allowed only while READY, PAUSED, or verifying stop')
        self._stop_detector.stop()
        self.get_logger().info(f'[MISSION] skipped tree={target.tree_id}')
        if self.core.state in (
                MissionState.NAVIGATING, MissionState.RETURNING_HOME):
            if not self._nav_client.server_is_ready():
                self._fail('Nav2 Action server is not ready after skip')
                return self._reply(response, False, self.core.last_error)
            self._start_navigation()
        else:
            self._publish_status()
        return self._reply(response, True, f'skipped {target.tree_id}')

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
        if (self._nav_handle is not None or self._spray_handle is not None or
                self._nav_pending or self._spray_pending):
            return self._reply(response, False, 'active goal has not settled')
        if not self.core.reset():
            return self._reply(response, False, 'reset requires a terminal state')
        self._restore_configured_mission_options()
        self._manual_return_home = False
        self._abort_and_home_requested = False
        self._abort_reset_sent = False
        self._recovery_return_home = False
        self._recovery_pause = False
        self._nav_timeout_canceling = False
        self._nav_timeout_cancel_deadline = None
        self._relay_failure_latched = False
        self._clear_nav_startup_retry()
        self._reset_docking_verification()
        self._publish_plan()
        self._publish_status()
        return self._reply(response, True, 'mission reset')

    @staticmethod
    def _reply(response, success, message):
        response.success = success
        response.message = message
        return response

    def _servers_ready(self):
        needs_spray = any(target.requires_arm for target in self.core.targets)
        return (
            self._nav_client.server_is_ready()
            and (not needs_spray or self._spray_client.server_is_ready())
            and (not getattr(self, '_require_relay_service', False)
                 or self._relay_client.service_is_ready()))

    def _clear_nav_startup_retry(self):
        self._initial_nav_started = None
        self._nav_retry_due = None

    def _begin_mission_navigation(self):
        self.core.start()
        self._reset_docking_verification()
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
            target = self.core.current_target
            if target is None:
                self._fail('no current navigation target')
                return
            if self._docking_retry_target_index != self.core.current_index:
                self._docking_retry_target_index = self.core.current_index
                self._docking_retry_count = 0
            x, y, yaw = target.docking_pose
            target_label = target.tree_id
        if self.core.state == MissionState.RETURNING_HOME:
            behavior_tree = getattr(self, '_route_nav_behavior_tree', '')
            navigation_profile = 'route'
        else:
            behavior_tree = (
                getattr(self, '_inspect_nav_behavior_tree', '')
                if target.requires_arm else
                getattr(self, '_route_nav_behavior_tree', ''))
            navigation_profile = 'inspect' if target.requires_arm else 'route'
        goal = NavigateToPose.Goal()
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.header.frame_id = self._map_frame
        self._set_pose(goal.pose.pose, x, y, yaw)
        if behavior_tree:
            goal.behavior_tree = behavior_tree
        self._nav_pending = True
        self._phase_started = self._now()
        future = self._nav_client.send_goal_async(goal)
        future.add_done_callback(self._nav_goal_response)
        self.get_logger().info(
            f'[NAV] sent target={target_label} profile={navigation_profile} '
            f'pose=({x:.2f},{y:.2f},{yaw:.2f})')
        self._publish_status()

    def _start_navigation(self):
        """Ensure wide spray is off for a disabled incoming segment first."""
        if self.core.state == MissionState.RETURNING_HOME:
            self._wide_motion_pending = False
            self._command_relay_best_effort(
                self._wide_relay_channel, False, 0.0,
                self._send_nav_goal, 'returning home: disable wide spray')
            return
        target = self.core.current_target
        if target is None:
            self._fail('no current navigation target')
            return
        self._wide_motion_pending = False
        if target.wide_spray_on_approach:
            self._send_nav_goal()
            return
        self._command_relay_best_effort(
            self._wide_relay_channel, False, 0.0,
            self._send_nav_goal,
            f'{target.tree_id}: approach has wide spray disabled')

    def _navigation_active(self):
        return self.core.state in (
            MissionState.NAVIGATING, MissionState.RETURNING_HOME)

    def _begin_stop_verification(self):
        if self.core.state != MissionState.VERIFYING_STOP:
            return
        self._localization_recovery_started = None
        self._last_docking_log_state = None
        self._phase_started = self._now()
        self._stop_detector.start(self._phase_started)
        self._publish_status()

    def _skip_navigation_point(self, reason):
        """Continue after a Nav2 result that explicitly ended the goal."""
        self._wide_motion_pending = False

        def skip_after_wide_off():
            target = self.core.current_target
            if target is None or not self.core.skip_current(
                    self._return_home_after_finish, reason):
                self._fail(reason)
                return
            self._stop_detector.stop()
            self.get_logger().warning(
                f'[MISSION] skipped point={target.tree_id}: {reason}')
            self._continue_after_point()

        # A failed navigation can occur with channel 1 active.  The off
        # command must be issued before this route point is discarded.
        self._command_relay_best_effort(
            self._wide_relay_channel, False, 0.0, skip_after_wide_off,
            f'navigation failure: disable wide spray before skip ({reason})')

    def _finish_noninspect_point(self):
        target = self.core.current_target
        if target is None or not self.core.point_succeeded(
                self._return_home_after_finish, 'route point completed'):
            self._fail('cannot complete non-inspect route point')
            return
        self.get_logger().info(f'[MISSION] completed point={target.tree_id}')
        self._continue_after_point()

    def _continue_after_point(self):
        if self.core.state in {
                MissionState.NAVIGATING, MissionState.RETURNING_HOME}:
            self._start_navigation()
            return
        self._command_all_relays_off()
        if self.core.state == MissionState.MISSION_COMPLETED:
            terminal = (
                'MISSION_COMPLETED' if self.core.all_targets_completed
                else 'MISSION_FINISHED_INCOMPLETE')
            self.get_logger().info(
                f'[MISSION] {terminal} total={len(self.core.targets)} '
                f'completed={self.core.completed_targets} '
                f'partial={self.core.partial_targets} '
                f'skipped={self.core.skipped_targets}')
        self._publish_status()

    def _relay_command_failed(self, *, channel, enabled, context, detail, critical):
        """Log a relay failure and fail closed when the relay is required."""
        message = (
            f'channel={channel} enabled={enabled} context={context}: {detail}')
        if critical and getattr(self, '_require_relay_service', False):
            self._relay_required_failure(message)
            return False
        self.get_logger().warning(
            f'[MISSION][WARN][RELAY] {message}; continuing')
        return True

    def _relay_required_failure(self, detail):
        """Fail closed without recursively issuing another relay request.

        A failed ON command means the visible treatment was not delivered; a
        failed OFF command leaves the physical output state unknown.  In both
        cases the only safe route action is to stop the vehicle, cancel active
        work and latch the mission in FAILED.  We deliberately do not call
        ``_command_all_relays_off`` here because the service itself is the
        failing dependency.
        """
        if getattr(self, '_relay_failure_latched', False):
            return
        self._relay_failure_latched = True
        self._wide_motion_pending = False
        self._stop_detector.stop()
        self._clear_nav_startup_retry()
        self._cancel_nav_goal()
        self._cancel_spray_goal()
        self._publish_motion_command('stop')
        message = f'required relay command failed: {detail}'
        self.core.fail(message)
        self.get_logger().error(f'[MISSION][RELAY] {message}')
        self._publish_status()

    def _command_relay_best_effort(
            self, channel, enabled, duration, continuation, context,
            *, critical=True):
        """Issue a non-blocking relay request with optional fail-closed policy."""
        channel = int(channel)
        enabled = bool(enabled)
        duration = float(duration)
        if not self._relay_client.service_is_ready():
            if (self._relay_command_failed(
                    channel=channel, enabled=enabled, context=context,
                    detail='service unavailable', critical=critical)
                    and continuation is not None):
                continuation()
            return
        request = SetRelay.Request()
        request.channel = channel
        request.enabled = enabled
        request.duration = duration
        try:
            future = self._relay_client.call_async(request)
        except Exception as error:
            if (self._relay_command_failed(
                    channel=channel, enabled=enabled, context=context,
                    detail=str(error), critical=critical)
                    and continuation is not None):
                continuation()
            return

        def done(result_future):
            try:
                response = result_future.result()
                if response is None or not response.success:
                    message = '' if response is None else response.message
                    continue_route = self._relay_command_failed(
                        channel=channel, enabled=enabled, context=context,
                        detail=message or 'request rejected', critical=critical)
                else:
                    if channel == self._wide_relay_channel:
                        self._wide_relay_enabled = enabled
                        self._publish_wide_active()
                    state = 'ON' if enabled else 'OFF'
                    self.get_logger().info(
                        f'[RELAY] channel={channel} state={state} '
                        f'duration={duration:.2f}s context={context}')
                    continue_route = True
            except Exception as error:
                continue_route = self._relay_command_failed(
                    channel=channel, enabled=enabled, context=context,
                    detail=str(error), critical=critical)
            if continue_route and continuation is not None:
                continuation()

        future.add_done_callback(done)

    def _publish_wide_active(self):
        """Publish the last successfully confirmed wide-relay command."""
        self._wide_active_pub.publish(Bool(data=self._wide_relay_enabled))

    def _command_all_relays_off(self):
        self._wide_motion_pending = False
        self._command_relay_best_effort(
            self._wide_relay_channel, False, 0.0, None,
            'mission shutdown: disable wide spray', critical=False)
        self._command_relay_best_effort(
            self._arm_relay_channel, False, 0.0, None,
            'mission shutdown: disable arm spray', critical=False)

    def _tick_wide_spray_motion(self, now):
        if not self._wide_motion_pending:
            return
        if self.core.state != MissionState.NAVIGATING:
            self._wide_motion_pending = False
            return
        if now >= self._wide_motion_deadline:
            self._wide_motion_pending = False
            self.get_logger().warning(
                '[MISSION][WARN][RELAY] vehicle did not start moving before '
                'wide spray timeout; continuing with channel 1 off')
            return
        if self._last_odom_at is None or now - self._last_odom_at > float(
                self.get_parameter('odom_stale_timeout_sec').value):
            return
        if self._latest_linear_speed < self._wide_motion_linear_threshold:
            return
        self._wide_motion_pending = False
        target = self.core.current_target
        context = (
            'vehicle motion confirmed; enable wide spray'
            if target is None else
            f'{target.tree_id}: vehicle motion confirmed; enable wide spray')
        self._command_relay_best_effort(
            self._wide_relay_channel, True, 0.0, None, context)

    def _publish_motion_command(self, command):
        self._motion_command_pub.publish(String(data=str(command)))

    def _advance_abort_and_home(self):
        if not self._abort_and_home_requested or self._abort_reset_sent:
            return
        if any((self._nav_pending, self._spray_pending,
                self._nav_handle is not None, self._spray_handle is not None)):
            return
        self._abort_reset_sent = True
        self._publish_motion_command('reset')
        self.core.last_error = 'abort_and_home: motion_control reset requested'
        self._publish_status()

    def _pause_for_recovery(self, message):
        returning_home = self.core.state == MissionState.RETURNING_HOME
        if not self.core.pause_for_recovery():
            self._fail(message)
            return
        self._recovery_return_home = returning_home
        self._recovery_pause = True
        self._stop_detector.stop()
        self._clear_nav_startup_retry()
        self._cancel_nav_goal()
        self._command_all_relays_off()
        self.core.last_error = str(message)
        self._publish_status()

    def _localization_ready_for_resume(self):
        pose = self._docking_pose_for_quality()
        if pose is None:
            return False
        received_at, _x, _y, _yaw, position_stddev, yaw_stddev = pose
        now = self._now()
        return (
            now - received_at <= self._localization_max_age and
            position_stddev <= self._max_localization_position_stddev and
            yaw_stddev <= self._max_localization_yaw_stddev)

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
              self.core.current_target is not None and
              self.core.current_target.wide_spray_on_approach):
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
            if (wrapped.status == GoalStatus.STATUS_ABORTED and
                    getattr(self, '_accept_aborted_near_goal', False)):
                status, details = self._docking_quality(self._now())
                if status == 'ok':
                    self.get_logger().warning(
                        '[NAV] accepted aborted near-goal result after '
                        f"docking gate: target={details['target']} "
                        f"position_error={details['position_error']:.3f}m "
                        f"yaw_error={details['yaw_error']:.3f}rad")
                    self._navigation_arrived(recovered=True)
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

    def _navigation_arrived(self, recovered=False):
        """Enter the existing relay/stop gate after final pose convergence."""
        self._wide_motion_pending = False
        if self.core.state == MissionState.NAVIGATING:
            if not self.core.nav_succeeded():
                self._fail('cannot enter stop verification after navigation result')
                return
        elif self.core.state != MissionState.VERIFYING_STOP:
            self._fail('cannot enter stop verification from current mission state')
            return
        target = self.core.current_target
        if target is None:
            self._fail('navigation completed without a route point')
            return
        if not recovered:
            self.get_logger().info(f'[NAV] succeeded point={target.tree_id}')
        self._command_relay_best_effort(
            self._wide_relay_channel, False, 0.0,
            self._begin_stop_verification,
            f'{target.tree_id}: disable wide spray at stop')

    def _on_odom(self, message):
        linear = math.hypot(message.twist.twist.linear.x, message.twist.twist.linear.y)
        angular = abs(message.twist.twist.angular.z)
        now = self._now()
        self._last_odom_at = now
        self._latest_linear_speed = linear
        self._stop_detector.update(now, linear, angular)
        if self._docking_pose_source != 'odom':
            return
        pose = message.pose.pose
        yaw = self._yaw_from_quaternion(pose.orientation)
        if yaw is None or not all(math.isfinite(value) for value in (
                pose.position.x, pose.position.y, yaw)):
            self._odom_docking_pose = None
            return
        # system_sim publishes an identity map->odom transform.  This source
        # is deliberately opt-in and must not be selected on real hardware.
        self._odom_docking_pose = (
            now, float(pose.position.x), float(pose.position.y), yaw,
            0.0, 0.0,
        )

    def _on_motion_control_state(self, message):
        self._motion_control_state = str(message.data)
        if self._abort_and_home_requested and self._abort_reset_sent:
            if self._motion_control_state in {'HOME_LOCKED', 'RESET_FAILED'}:
                self.core.last_error = (
                    'abort_and_home: arm HOME complete'
                    if self._motion_control_state == 'HOME_LOCKED'
                    else 'abort_and_home: arm HOME reset failed')
                self._publish_status()

    @staticmethod
    def _yaw_from_quaternion(quaternion):
        norm = math.sqrt(
            quaternion.x * quaternion.x + quaternion.y * quaternion.y +
            quaternion.z * quaternion.z + quaternion.w * quaternion.w)
        if not math.isfinite(norm) or norm < 1e-6:
            return None
        x = quaternion.x / norm
        y = quaternion.y / norm
        z = quaternion.z / norm
        w = quaternion.w / norm
        return math.atan2(
            2.0 * (w * z + x * y),
            1.0 - 2.0 * (y * y + z * z))

    @staticmethod
    def _angle_error(actual, desired):
        return abs(math.atan2(
            math.sin(actual - desired), math.cos(actual - desired)))

    def _on_localization_pose(self, message):
        """缓存AMCL在map中的实时位姿及协方差质量。"""
        if str(message.header.frame_id).strip() != self._map_frame:
            self._localization_pose = None
            return
        pose = message.pose.pose
        yaw = self._yaw_from_quaternion(pose.orientation)
        covariance = message.pose.covariance
        values = (pose.position.x, pose.position.y, yaw)
        if yaw is None or not all(math.isfinite(value) for value in values):
            self._localization_pose = None
            return
        try:
            x_variance = float(covariance[0])
            y_variance = float(covariance[7])
            yaw_variance = float(covariance[35])
        except (IndexError, TypeError, ValueError):
            self._localization_pose = None
            return
        if not all(math.isfinite(value) and value >= 0.0 for value in (
                x_variance, y_variance, yaw_variance)):
            self._localization_pose = None
            return
        self._localization_pose = (
            self._now(), float(pose.position.x), float(pose.position.y), yaw,
            max(math.sqrt(x_variance), math.sqrt(y_variance)),
            math.sqrt(yaw_variance),
        )

    def _docking_pose_for_quality(self):
        if getattr(self, '_docking_pose_source', 'localization') == 'odom':
            return getattr(self, '_odom_docking_pose', None)
        return getattr(self, '_localization_pose', None)

    def _docking_quality(self, now):
        target = self.core.current_target
        pose = self._docking_pose_for_quality()
        if target is None or pose is None:
            return 'unavailable', None
        received_at, x, y, yaw, position_stddev, yaw_stddev = pose
        age = max(0.0, now - received_at)
        desired = target.docking_pose
        desired_arm = arm_base_xy(
            desired, self._arm_base_forward_offset,
            self._arm_base_left_offset)
        actual_arm = arm_base_xy(
            (x, y, yaw), self._arm_base_forward_offset,
            self._arm_base_left_offset)
        position_error = math.hypot(
            actual_arm[0] - desired_arm[0],
            actual_arm[1] - desired_arm[1])
        yaw_error = self._angle_error(yaw, desired[2])
        details = {
            'target': target.tree_id,
            'source': getattr(self, '_docking_pose_source', 'localization'),
            'desired': desired,
            'actual': (x, y, yaw),
            'arm_desired': desired_arm,
            'arm_actual': actual_arm,
            'position_error': position_error,
            'yaw_error': yaw_error,
            'position_stddev': position_stddev,
            'yaw_stddev': yaw_stddev,
            'age': age,
        }
        if age > self._localization_max_age:
            return 'stale', details
        if (position_stddev > self._max_localization_position_stddev or
                yaw_stddev > self._max_localization_yaw_stddev):
            return 'uncertain', details
        if (position_error > self._max_docking_position_error or
                yaw_error > self._max_docking_yaw_error):
            return 'outside_tolerance', details
        return 'ok', details

    def _log_docking_quality(self, status, details):
        if status == getattr(self, '_last_docking_log_state', None):
            return
        self._last_docking_log_state = status
        if details is None:
            source = getattr(self, '_docking_pose_source', 'localization')
            self.get_logger().warn(
                f'[DOCK] {source} pose is unavailable; arm motion remains inhibited')
            return
        desired = details['desired']
        actual = details['actual']
        arm_desired = details['arm_desired']
        arm_actual = details['arm_actual']
        message = (
            f"[DOCK] target={details['target']} status={status} "
            f"source={details['source']} "
            f"arm_desired=({arm_desired[0]:.3f},{arm_desired[1]:.3f}) "
            f"arm_actual=({arm_actual[0]:.3f},{arm_actual[1]:.3f}) "
            f"base_goal=({desired[0]:.3f},{desired[1]:.3f},{desired[2]:.3f}) "
            f"base_actual=({actual[0]:.3f},{actual[1]:.3f},{actual[2]:.3f}) "
            f"position_error={details['position_error']:.3f}m "
            f"yaw_error={details['yaw_error']:.3f}rad "
            f"position_stddev={details['position_stddev']:.3f}m "
            f"yaw_stddev={details['yaw_stddev']:.3f}rad "
            f"nav_goal_tolerance=(xy={getattr(self, '_nav_goal_xy_tolerance', float('nan')):.2f}m,"
            f"yaw={getattr(self, '_nav_goal_yaw_tolerance', float('nan')):.2f}rad) "
            f"mission_gate=(xy={getattr(self, '_max_docking_position_error', float('nan')):.2f}m,"
            f"yaw={getattr(self, '_max_docking_yaw_error', float('nan')):.2f}rad) "
            f"age={details['age']:.2f}s "
            f"attempt={self._docking_retry_count + 1}/"
            f"{self._docking_retry_limit + 1}")
        if status == 'ok':
            self.get_logger().info(message)
        else:
            self.get_logger().warn(message)

    def _verify_docking_quality(self, now):
        """Return true only after stop, AMCL quality and docking error pass."""
        status, details = self._docking_quality(now)
        self._log_docking_quality(status, details)
        if status == 'ok':
            self._localization_recovery_started = None
            return True
        if status in {'unavailable', 'stale', 'uncertain'}:
            if self._localization_recovery_started is None:
                self._localization_recovery_started = now
            elif (now - self._localization_recovery_started >=
                  self._localization_recovery_timeout):
                self._pause_for_recovery(
                    f'docking localization did not recover: {status}')
            return False
        self._localization_recovery_started = None
        if self._docking_retry_count < self._docking_retry_limit:
            self._docking_retry_count += 1
            self._stop_detector.stop()
            if not self.core.retry_navigation():
                self._fail('cannot retry docking from current mission state')
                return False
            self._last_docking_log_state = None
            self.get_logger().warn(
                f'[DOCK] retrying target={self.core.current_target.tree_id} '
                f'attempt={self._docking_retry_count + 1}/'
                f'{self._docking_retry_limit + 1}')
            self._send_nav_goal()
            return False
        self._pause_for_recovery(
            'docking pose remains outside tolerance after retry')
        return False

    def _send_spray_goal(self):
        """构建并发送机械臂 Action 目标。"""
        target = self.core.current_target
        goal = ExecuteSpray.Goal()
        goal.mission_id = self.core.mission_id
        goal.tree_id = target.tree_id
        goal.spray_duration = target.spray_duration
        tree_x, tree_y, tree_z = self._tree_hint(target)
        goal.tree_hint.header.stamp = self.get_clock().now().to_msg()
        goal.tree_hint.header.frame_id = self._map_frame
        goal.tree_hint.point.x = tree_x
        goal.tree_hint.point.y = tree_y
        goal.tree_hint.point.z = tree_z
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

    @staticmethod
    def _tree_hint(target):
        return target.x, target.y, target.z

    def _skip_arm_point(self, reason):
        target = self.core.current_target
        if target is None or not self.core.skip_current(
                self._return_home_after_finish, reason):
            self._fail(reason)
            return
        self._manual_return_home = False
        self.get_logger().warning(
            f'[MISSION] skipped arm point={target.tree_id}: {reason}')
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
        finished = self.core.current_target.tree_id
        self._manual_return_home = False
        if result.error_code == ExecuteSpray.Result.INSPECTED_NO_DISEASE:
            outcome = 'inspected without disease'
            self.core.arm_succeeded(
                self._return_home_after_finish, result.message)
        elif result.error_code == ExecuteSpray.Result.PARTIAL_SUCCESS:
            outcome = 'partially sprayed'
            self.core.arm_partial(
                result.message, self._return_home_after_finish)
            self.get_logger().warn(
                f'[MISSION] partial tree={finished}: {result.message}')
        else:
            outcome = 'sprayed'
            self.core.arm_succeeded(
                self._return_home_after_finish, result.message)
        self.get_logger().info(
            f'[MISSION] {outcome} tree={finished} '
            f'processed={self.core.processed_targets}/'
            f'{len(self.core.targets)} completed={self.core.completed_targets} '
            f'partial={self.core.partial_targets} '
            f'skipped={self.core.skipped_targets}: {result.message}')
        if self.core.state == MissionState.NAVIGATING:
            self._start_navigation()
        elif self.core.state == MissionState.RETURNING_HOME:
            self._start_navigation()
        else:
            if self.core.state == MissionState.MISSION_COMPLETED:
                self.get_logger().info(
                    f'[MISSION] MISSION_COMPLETED targets={len(self.core.targets)} '
                    f'completed={self.core.completed_targets} '
                    f'partial={self.core.partial_targets} '
                    f'skipped={self.core.skipped_targets}')
            self._continue_after_point()

    # ---------- 100ms 调度看门狗 (Tick) ----------
    def _tick(self):
        """
        核心调度步进逻辑。每 0.1 秒触发一次，用于检测：
        1. Nav2 或机械臂的全局超时。
        2. 停稳检测的状态转换。
        3. 机械臂反馈进度的卡死检查 (progress_timeout)。
        """
        now = self._now()
        self._advance_abort_and_home()
        if self._abort_and_home_requested:
            return
        self._tick_wide_spray_motion(now)
        if self._nav_retry_due is not None:
            if not self._navigation_active():
                self._clear_nav_startup_retry()
            elif now >= self._nav_retry_due:
                self._nav_retry_due = None
                self._start_navigation()
            return
        if self._navigation_active() and self._phase_started is not None:
            if now - self._phase_started >= self._nav_timeout:
                if not self._nav_timeout_canceling:
                    self._nav_timeout_canceling = True
                    self._nav_timeout_cancel_deadline = now + 5.0
                    self._phase_started = None
                    self._cancel_nav_goal()
                    self.get_logger().warning(
                        '[NAV] goal timed out; canceling before skipping point')
        if self._nav_timeout_canceling:
            if (self._nav_timeout_cancel_deadline is not None and
                    now >= self._nav_timeout_cancel_deadline):
                self._nav_timeout_canceling = False
                self._nav_timeout_cancel_deadline = None
                self._pause_for_recovery(
                    'Nav2 timeout cancellation did not settle; relocalize and resume')
            return
        if self.core.state == MissionState.VERIFYING_STOP:
            status = self._stop_detector.status(now)
            if status == StopDetector.STABLE:
                target = self.core.current_target
                if target is None:
                    self._fail('stop verified without a route point')
                elif (not target.requires_arm or
                      not getattr(self, '_require_docking_quality', False) or
                      self._verify_docking_quality(now)):
                    self._stop_detector.stop()
                    self.core.stop_verified()
                    self.get_logger().info(
                        '[STOP_CHECK] vehicle is stable and docking quality passed')
                    if self.core.state == MissionState.ARM_SPRAYING:
                        self._send_spray_goal()
                    elif self.core.state == MissionState.DWELLING:
                        self._phase_started = now
                        if target.dwell_time_sec <= 0.0:
                            self._finish_noninspect_point()
                        else:
                            self._publish_status()
            elif status in (StopDetector.STALE, StopDetector.TIMEOUT):
                target = self.core.current_target
                if target is not None and target.requires_arm:
                    # Nav2 has already reported arrival.  Without a verified
                    # stationary vehicle the arm must not move, but this one
                    # inspection point must not terminate the remaining route.
                    self._skip_arm_point(
                        f'odom stop verification failed: {status}')
                else:
                    self._skip_navigation_point(
                        f'odom stop verification failed: {status}')
        elif self.core.state == MissionState.DWELLING:
            target = self.core.current_target
            if target is None:
                self._fail('dwell without a route point')
            elif (self._phase_started is not None and
                  now - self._phase_started >= target.dwell_time_sec):
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
                not self.core.all_targets_completed)
            else self.core.state.name)
        target = self.core.current_target
        message.current_tree_id = target.tree_id if target else ''
        message.current_index = self.core.current_index
        message.total_targets = len(self.core.targets)
        message.completed_targets = self.core.completed_targets
        message.skipped_targets = self.core.skipped_targets
        for index, target_item in enumerate(self.core.targets):
            target_status = MissionTargetStatus()
            target_status.tree_id = target_item.tree_id
            outcome = self.core.target_outcomes[index]
            if outcome == MissionCore.COMPLETED:
                target_status.state = MissionTargetStatus.COMPLETED
            elif outcome == MissionCore.SKIPPED:
                target_status.state = MissionTargetStatus.SKIPPED
            elif outcome in {MissionCore.PARTIAL, MissionCore.FAILED}:
                target_status.state = MissionTargetStatus.FAILED
                target_status.message = self.core.target_messages[index]
            elif index == self.core.current_index:
                target_status.state = MissionTargetStatus.CURRENT
            else:
                target_status.state = MissionTargetStatus.PENDING
            target_status.state_text = (
                'CURRENT' if target_status.state == MissionTargetStatus.CURRENT
                else outcome)
            message.target_statuses.append(target_status)
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
        message.return_home_after_finish = self._return_home_after_finish
        self._set_pose(message.home_pose, *self._home_pose)
        for target in self.core.targets:
            item = MissionTargetPlan()
            item.target_id = target.tree_id
            item.tree_hint.x = target.x
            item.tree_hint.y = target.y
            item.tree_hint.z = target.z
            item.spray_duration = target.spray_duration
            item.tree_x_m = target.tree_x_m
            item.tree_y_m = target.tree_y_m
            self._set_pose(item.docking_pose, *target.docking_pose)
            message.targets.append(item)
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
