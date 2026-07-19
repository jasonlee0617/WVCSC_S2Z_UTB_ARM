"""空地协同任务编排节点。

节点接收无人机在 ``map`` 坐标系发布的病树列表，为每棵树计算道路停靠位姿，依次
调用 Nav2 ``NavigateToPose``、基于 ``/odom`` 的连续停稳检测和机械臂
``ExecuteSpray``。它只负责树级顺序、超时、取消、返回 HOME 和公开状态；病果级
观察/识别/喷洒由 ``wvcsc_arm_task`` 负责。

回调由双线程执行器处理，但所有状态转移都委托给 ``MissionCore``。只要存在跳过或
部分喷洒，剩余树仍会继续执行，但最终状态必须为 ``FAILED``，不会误报为任务完成。
"""

import math

from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger
from wvcsc_interfaces.action import ExecuteSpray
from wvcsc_interfaces.msg import (
    DiseaseTreeArray,
    MissionPlan,
    MissionStatus,
    MissionTargetPlan,
    MissionTargetStatus,
)
from wvcsc_interfaces.srv import LoadManualMission

from .core import (
    DEFAULT_DOCKING_LATERAL_OFFSET,
    MissionCore,
    MissionState,
    StopDetector,
    Target,
    docking_pose,
    manual_tree_hint,
    navigation_pose,
)


class MissionManager(Node):
    """串联 Mock/真实 UAV、Nav2 与机械臂 Action 的事件驱动状态机外壳。"""

    def __init__(self, **kwargs):
        super().__init__('mission_manager', **kwargs)
        self._declare_parameters()
        self.core = MissionCore()
        self._auto_start = bool(self.get_parameter('auto_start').value)
        self._map_frame = str(self.get_parameter('map_frame').value)
        self._road_center_y = float(self.get_parameter('road_center_y').value)
        self._road_yaw = float(self.get_parameter('road_yaw').value)
        self._docking_lateral_offset = float(
            self.get_parameter('docking_lateral_offset').value)
        self._manual_tree_standoff = float(
            self.get_parameter('manual_tree_standoff').value)
        self._manual_tree_base_z = float(
            self.get_parameter('manual_tree_base_z').value)
        self._nav_timeout = float(self.get_parameter('nav_goal_timeout_sec').value)
        self._nav_startup_retry_timeout = float(
            self.get_parameter('nav_startup_retry_timeout_sec').value)
        self._nav_startup_retry_interval = float(
            self.get_parameter('nav_startup_retry_interval_sec').value)
        self._spray_timeout = float(self.get_parameter('spray_goal_timeout_sec').value)
        self._spray_progress_timeout = float(
            self.get_parameter('spray_progress_timeout_sec').value)
        self._return_home_after_finish = bool(
            self.get_parameter('return_home_after_finish').value)
        self._home_pose = (
            float(self.get_parameter('home_x').value),
            float(self.get_parameter('home_y').value),
            float(self.get_parameter('home_yaw').value),
        )
        if not all(math.isfinite(value) for value in self._home_pose):
            raise ValueError('home pose must contain finite values')
        if (not math.isfinite(self._nav_startup_retry_timeout) or
                not math.isfinite(self._nav_startup_retry_interval) or
                self._nav_startup_retry_timeout <= 0.0 or
                self._nav_startup_retry_interval <= 0.0):
            raise ValueError('initial Nav2 retry timing must be finite and positive')
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

        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._status_pub = self.create_publisher(
            MissionStatus, '/mission/status', latched)
        self._plan_pub = self.create_publisher(
            MissionPlan, '/mission/plan', latched)
        self.create_subscription(
            DiseaseTreeArray, '/uav/disease_trees', self._on_mission, latched)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)

        self._nav_client = ActionClient(
            self, NavigateToPose, str(self.get_parameter('nav_action_name').value))
        self._spray_client = ActionClient(
            self, ExecuteSpray, str(self.get_parameter('spray_action_name').value))
        self._nav_handle = None
        self._spray_handle = None
        self._nav_pending = False
        self._spray_pending = False
        self._phase_started = None
        self._initial_nav_started = None
        self._nav_retry_due = None
        self._spray_last_progress = None
        self._manual_return_home = False

        self.create_service(Trigger, '/mission/start', self._start)
        self.create_service(Trigger, '/mission/pause', self._pause)
        self.create_service(Trigger, '/mission/resume', self._resume)
        self.create_service(
            Trigger, '/mission/skip_current', self._skip_current)
        self.create_service(
            Trigger, '/mission/return_home', self._return_home)
        self.create_service(Trigger, '/mission/cancel', self._cancel)
        self.create_service(Trigger, '/mission/reset', self._reset)
        self.create_service(
            LoadManualMission, '/mission/load_manual', self._load_manual)
        self.create_timer(0.1, self._tick)
        self.create_timer(0.5, self._publish_status)
        self._publish_status()
        self._publish_plan()

    def _declare_parameters(self):
        parameters = {
            'auto_start': False,
            'map_frame': 'map',
            'nav_action_name': '/navigate_to_pose',
            'spray_action_name': '/arm/execute_spray',
            'road_center_y': 0.0,
            'road_yaw': 0.0,
            'docking_lateral_offset': DEFAULT_DOCKING_LATERAL_OFFSET,
            'manual_tree_standoff': 1.5,
            'manual_tree_base_z': 0.0,
            'nav_goal_timeout_sec': 120.0,
            # An action server can exist before Nav2's lifecycle nodes are active.
            # Bound retries make auto-start independent of launch ordering.
            'nav_startup_retry_timeout_sec': 30.0,
            'nav_startup_retry_interval_sec': 0.5,
            'spray_goal_timeout_sec': 180.0,
            'spray_progress_timeout_sec': 30.0,
            'return_home_after_finish': False,
            'home_x': 0.0,
            'home_y': 0.0,
            'home_yaw': 0.0,
            'linear_stop_threshold': 0.03,
            'angular_stop_threshold': 0.03,
            'stop_stable_duration_sec': 1.0,
            'odom_stale_timeout_sec': 1.0,
            'stop_verify_timeout_sec': 5.0,
            'confidence_threshold': 0.5,
            'max_targets': 20,
            'max_abs_coordinate': 50.0,
            'min_spray_duration': 0.2,
            'max_spray_duration': 10.0,
        }
        for name, default in parameters.items():
            self.declare_parameter(name, default)

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_mission(self, message):
        """校验并装载一批 map 坐标病树；运行中的任务不会被新消息覆盖。"""
        try:
            targets = self._validate_message(message)
            outcome = self.core.load(message.mission_id.strip(), targets)
        except ValueError as error:
            self.get_logger().error(f'[MISSION] rejected task list: {error}')
            return
        if outcome == 'accepted':
            self._restore_configured_mission_options()
            self._manual_return_home = False
            self._clear_nav_startup_retry()
            self.get_logger().info(
                f'[MISSION] accepted mission={self.core.mission_id} '
                f'targets={len(self.core.targets)}')
            self._publish_plan()
            self._publish_status()
        elif outcome == 'duplicate':
            self.get_logger().info(
                f'[MISSION] ignored duplicate mission={message.mission_id}')
        else:
            self.get_logger().warn(
                f'[MISSION] rejected new mission={message.mission_id}: busy')

    def _validate_message(self, message):
        if message.header.frame_id != self._map_frame:
            raise ValueError(f'frame must be {self._map_frame}')
        if message.source_mode not in ('mock', 'replay', 'live'):
            raise ValueError('source_mode must be mock, replay or live')
        if not message.mission_id.strip() or not message.trees:
            raise ValueError('mission_id and targets are required')
        max_targets = int(self.get_parameter('max_targets').value)
        if len(message.trees) > max_targets:
            raise ValueError(f'target count exceeds limit {max_targets}')
        threshold = float(self.get_parameter('confidence_threshold').value)
        bound = float(self.get_parameter('max_abs_coordinate').value)
        min_duration = float(self.get_parameter('min_spray_duration').value)
        max_duration = float(self.get_parameter('max_spray_duration').value)
        seen = set()
        targets = []
        for tree in message.trees:
            values = (tree.position.x, tree.position.y, tree.position.z,
                      tree.confidence, tree.spray_duration)
            if not tree.tree_id.strip() or tree.tree_id in seen:
                raise ValueError('tree_id must be non-empty and unique')
            if not all(math.isfinite(value) for value in values):
                raise ValueError(f'{tree.tree_id}: non-finite value')
            if tree.confidence < threshold or tree.confidence > 1.0:
                raise ValueError(f'{tree.tree_id}: confidence out of range')
            if tree.spray_side not in ('left', 'right'):
                raise ValueError(f'{tree.tree_id}: invalid spray_side')
            if not min_duration <= tree.spray_duration <= max_duration:
                raise ValueError(f'{tree.tree_id}: spray_duration out of range')
            if abs(tree.position.x) > bound or abs(tree.position.y) > bound:
                raise ValueError(f'{tree.tree_id}: position out of bounds')
            seen.add(tree.tree_id)
            target = Target(
                tree.tree_id, tree.position.x, tree.position.y, tree.position.z,
                tree.confidence, tree.spray_side, tree.spray_duration,
                tree.evidence_uri)
            docking_pose(
                target, self._road_center_y, self._road_yaw,
                self._docking_lateral_offset)
            targets.append(target)
        return targets

    def _load_manual(self, request, response):
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
        self._clear_nav_startup_retry()
        self.get_logger().info(
            f'[MISSION] accepted manual mission={self.core.mission_id} '
            f'targets={len(self.core.targets)}')
        self._publish_plan()
        self._publish_status()
        return self._reply(response, True, 'manual mission loaded')

    def _validate_manual_request(self, request):
        if request.header.frame_id != self._map_frame:
            raise ValueError(f'frame must be {self._map_frame}')
        if not request.mission_id.strip() or not request.targets:
            raise ValueError('mission_id and targets are required')
        max_targets = int(self.get_parameter('max_targets').value)
        if len(request.targets) > max_targets:
            raise ValueError(f'target count exceeds limit {max_targets}')
        home_pose = MissionManager._pose_to_xy_yaw(request.home_pose, 'home')
        bound = float(self.get_parameter('max_abs_coordinate').value)
        if abs(home_pose[0]) > bound or abs(home_pose[1]) > bound:
            raise ValueError('home: position out of bounds')
        min_duration = float(self.get_parameter('min_spray_duration').value)
        max_duration = float(self.get_parameter('max_spray_duration').value)
        seen = set()
        targets = []
        for item in request.targets:
            if not item.target_id.strip() or item.target_id in seen:
                raise ValueError('target_id must be non-empty and unique')
            if item.spray_side not in ('left', 'right'):
                raise ValueError(f'{item.target_id}: invalid spray_side')
            if not min_duration <= item.spray_duration <= max_duration:
                raise ValueError(f'{item.target_id}: spray_duration out of range')
            x, y, yaw = MissionManager._pose_to_xy_yaw(
                item.docking_pose, item.target_id)
            if abs(x) > bound or abs(y) > bound:
                raise ValueError(f'{item.target_id}: position out of bounds')
            seen.add(item.target_id)
            targets.append(Target(
                item.target_id, x, y, item.docking_pose.position.z,
                1.0, item.spray_side, item.spray_duration,
                f'manual://rviz/{item.target_id}', (x, y, yaw)))
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

    def _start(self, _request, response):
        if self.core.state != MissionState.READY:
            return self._reply(response, False, 'mission is not READY')
        if not self._servers_ready():
            return self._reply(response, False, 'Nav2 or spray Action server is not ready')
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
        self.core.resume()
        self._send_nav_goal()
        return self._reply(response, True, 'mission resumed')

    def _cancel(self, _request, response):
        if not self.core.cancel():
            return self._reply(response, False, 'mission cannot be canceled in this state')
        self._stop_detector.stop()
        self._cancel_nav_goal()
        self._cancel_spray_goal()
        self._publish_status()
        return self._reply(response, True, 'mission canceled')

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
            self._send_nav_goal()
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
        self._send_nav_goal()
        return self._reply(response, True, 'return home started')

    def _reset(self, _request, response):
        if (self._nav_handle is not None or self._spray_handle is not None or
                self._nav_pending or self._spray_pending):
            return self._reply(response, False, 'active goal has not settled')
        if not self.core.reset():
            return self._reply(response, False, 'reset requires a terminal state')
        self._restore_configured_mission_options()
        self._manual_return_home = False
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
        return self._nav_client.server_is_ready() and self._spray_client.server_is_ready()

    def _clear_nav_startup_retry(self):
        self._initial_nav_started = None
        self._nav_retry_due = None

    def _begin_mission_navigation(self):
        """启动首个导航，并为 Nav2 生命周期激活窗口建立有界重试计时。"""
        self.core.start()
        self._initial_nav_started = self._now()
        self._nav_retry_due = None
        self._send_nav_goal()

    def _schedule_initial_nav_retry(self):
        """仅在首个 Goal 被未激活的 Nav2 拒绝时重试，避免掩盖运行期故障。"""
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
        """发送当前树或 HOME 的 Nav2 Goal，并启动本阶段超时计时。"""
        if self.core.state == MissionState.RETURNING_HOME:
            x, y, yaw = self._home_pose
            target_label = 'HOME'
        else:
            target = self.core.current_target
            if target is None:
                self._fail('no current navigation target')
                return
            x, y, yaw = navigation_pose(
                target, self._road_center_y, self._road_yaw,
                self._docking_lateral_offset)
            target_label = target.tree_id
        goal = NavigateToPose.Goal()
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.header.frame_id = self._map_frame
        self._set_pose(goal.pose.pose, x, y, yaw)
        self._nav_pending = True
        self._phase_started = self._now()
        future = self._nav_client.send_goal_async(goal)
        future.add_done_callback(self._nav_goal_response)
        self.get_logger().info(
            f'[NAV] sent target={target_label} pose=({x:.2f},{y:.2f},{yaw:.2f})')
        self._publish_status()

    def _navigation_active(self):
        return self.core.state in (
            MissionState.NAVIGATING, MissionState.RETURNING_HOME)

    def _nav_goal_response(self, future):
        self._nav_pending = False
        try:
            handle = future.result()
        except Exception as error:
            if self._navigation_active():
                self._fail(f'Nav2 goal send failed: {error}')
            return
        if handle is None or not handle.accepted:
            if self._schedule_initial_nav_retry():
                return
            if self._navigation_active():
                self._fail('Nav2 rejected the goal')
            return
        self._nav_handle = handle
        self._clear_nav_startup_retry()
        if not self._navigation_active():
            self._cancel_nav_goal()
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._nav_result)

    def _nav_result(self, future):
        """消费 Nav2 最终结果；到树后先进入停稳检测，而非立即启动机械臂。"""
        self._nav_handle = None
        try:
            wrapped = future.result()
        except Exception as error:
            if self._navigation_active():
                self._fail(f'Nav2 result failed: {error}')
            return
        if not self._navigation_active():
            self._publish_status()
            return
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
            self._fail(f'Nav2 failed with status {wrapped.status}')
            return
        if self.core.state == MissionState.RETURNING_HOME:
            self.core.home_succeeded(canceled=self._manual_return_home)
            self.get_logger().info(
                f'[NAV] HOME reached; state={self.core.state.name}')
            self._publish_status()
            return
        self.core.nav_succeeded()
        self._phase_started = self._now()
        self._stop_detector.start(self._phase_started)
        self.get_logger().info(
            f'[NAV] succeeded tree={self.core.current_target.tree_id}')
        self._publish_status()

    def _on_odom(self, message):
        linear = math.hypot(message.twist.twist.linear.x, message.twist.twist.linear.y)
        angular = abs(message.twist.twist.angular.z)
        self._stop_detector.update(self._now(), linear, angular)

    def _send_spray_goal(self):
        """把当前病树提示和作业参数交给机械臂，树级状态仍由本节点持有。"""
        target = self.core.current_target
        goal = ExecuteSpray.Goal()
        goal.mission_id = self.core.mission_id
        goal.tree_id = target.tree_id
        goal.spray_side = target.spray_side
        goal.spray_duration = target.spray_duration
        tree_x, tree_y, tree_z = self._tree_hint(target)
        goal.tree_hint.header.stamp = self.get_clock().now().to_msg()
        goal.tree_hint.header.frame_id = self._map_frame
        goal.tree_hint.point.x = tree_x
        goal.tree_hint.point.y = tree_y
        goal.tree_hint.point.z = tree_z
        self._spray_pending = True
        self._phase_started = self._now()
        self._spray_last_progress = self._phase_started
        future = self._spray_client.send_goal_async(
            goal, feedback_callback=self._spray_feedback)
        future.add_done_callback(self._spray_goal_response)
        self._publish_status()

    def _tree_hint(self, target):
        if target.docking_pose_override is None:
            return target.x, target.y, target.z
        return manual_tree_hint(
            target.docking_pose_override, target.spray_side,
            self._manual_tree_standoff, self._manual_tree_base_z)

    def _spray_goal_response(self, future):
        self._spray_pending = False
        try:
            handle = future.result()
        except Exception as error:
            if self.core.state == MissionState.ARM_SPRAYING:
                self._fail(f'spray goal send failed: {error}')
            return
        if handle is None or not handle.accepted:
            if self.core.state == MissionState.ARM_SPRAYING:
                self._fail('spray Action rejected the goal')
            return
        self._spray_handle = handle
        if self.core.state != MissionState.ARM_SPRAYING:
            self._cancel_spray_goal()
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._spray_result)

    def _spray_feedback(self, feedback_message):
        """仅用反馈刷新进展看门狗；阶段明细由机械臂节点负责显示。"""
        feedback = feedback_message.feedback
        if self.core.state == MissionState.ARM_SPRAYING:
            self._spray_last_progress = self._now()
        self.get_logger().debug(
            f'[ARM] {feedback.phase_text} progress={feedback.progress:.2f}')

    def _spray_result(self, future):
        """根据机械臂结果推进下一棵树；部分成功保留为明确告警。"""
        self._spray_handle = None
        self._spray_last_progress = None
        try:
            wrapped = future.result()
        except Exception as error:
            if self.core.state == MissionState.ARM_SPRAYING:
                self._fail(f'spray result failed: {error}')
            return
        if self.core.state != MissionState.ARM_SPRAYING:
            self._publish_status()
            return
        result = wrapped.result
        if (not result.success
                and result.error_code == ExecuteSpray.Result.VISION_FAILED):
            skipped = self.core.current_target.tree_id
            self._manual_return_home = False
            self.core.skip_current(self._return_home_after_finish)
            self.get_logger().info(
                f'[MISSION] skipped tree={skipped} '
                f'processed={self.core.processed_targets}/'
                f'{len(self.core.targets)} completed={self.core.completed_targets} '
                f'partial={self.core.partial_targets} '
                f'skipped={self.core.skipped_targets}: {result.message}')
            if self.core.state in {
                    MissionState.NAVIGATING,
                    MissionState.RETURNING_HOME}:
                self._send_nav_goal()
            else:
                self._publish_status()
            return
        if wrapped.status != GoalStatus.STATUS_SUCCEEDED or not result.success:
            self._fail(
                f'spray failed status={wrapped.status} code={result.error_code}: '
                f'{result.message}')
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
            self._send_nav_goal()
        elif self.core.state == MissionState.RETURNING_HOME:
            self._send_nav_goal()
        else:
            if self.core.state == MissionState.FAILED:
                self.get_logger().error(
                    '[MISSION] FAILED: mission contains incomplete tree results')
            elif self.core.state == MissionState.MISSION_COMPLETED:
                self.get_logger().info(
                    f'[MISSION] MISSION_COMPLETED targets={len(self.core.targets)} '
                    f'completed={self.core.completed_targets} '
                    f'partial={self.core.partial_targets} '
                    f'skipped={self.core.skipped_targets}')
            self._publish_status()

    def _tick(self):
        """100 ms 看门狗：自动启动、导航超时、停稳确认和机械臂进展超时。"""
        if self.core.state == MissionState.READY and self._auto_start and self._servers_ready():
            self._begin_mission_navigation()
            return
        now = self._now()
        if self._nav_retry_due is not None:
            if not self._navigation_active():
                self._clear_nav_startup_retry()
            elif now >= self._nav_retry_due:
                self._nav_retry_due = None
                self._send_nav_goal()
            return
        if self._navigation_active() and self._phase_started is not None:
            if now - self._phase_started >= self._nav_timeout:
                self._fail('Nav2 goal timed out')
        elif self.core.state == MissionState.VERIFYING_STOP:
            status = self._stop_detector.status(now)
            if status == StopDetector.STABLE:
                self._stop_detector.stop()
                self.core.stop_verified()
                self.get_logger().info('[STOP_CHECK] vehicle is stable')
                self._send_spray_goal()
            elif status in (StopDetector.STALE, StopDetector.TIMEOUT):
                self._fail(f'odom stop verification failed: {status}')
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
        self.get_logger().error(f'[MISSION] FAILED: {message}')
        self._publish_status()

    def _cancel_nav_goal(self):
        if self._nav_handle is not None:
            self._nav_handle.cancel_goal_async()

    def _cancel_spray_goal(self):
        if self._spray_handle is not None:
            self._spray_handle.cancel_goal_async()

    def _publish_status(self):
        """发布 Transient Local 任务快照，供 UI、感知节点和晚加入订阅者使用。"""
        message = MissionStatus()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self._map_frame
        message.mission_id = self.core.mission_id
        message.state = int(self.core.state)
        message.state_text = self.core.state.name
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
            item.target.tree_id = target.tree_id
            item.target.confidence = target.confidence
            item.target.position.x = target.x
            item.target.position.y = target.y
            item.target.position.z = target.z
            item.target.spray_side = target.spray_side
            item.target.spray_duration = target.spray_duration
            item.target.evidence_uri = target.evidence_uri
            self._set_pose(
                item.docking_pose,
                *navigation_pose(
                    target, self._road_center_y, self._road_yaw,
                    self._docking_lateral_offset),
            )
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
