import math
import os
import time

import pytest

os.environ['ROS_DOMAIN_ID'] = '82'
os.environ.setdefault('ROS_LOG_DIR', '/tmp/wvcsc_mission_test_logs')

from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger
from wvcsc_interfaces.action import ExecuteSpray
from wvcsc_interfaces.msg import ManualMissionPoint, MissionPlan, MissionStatus
from wvcsc_interfaces.srv import LoadManualMission

from wvcsc_mission_manager.mission_manager import MissionManager


class _FakeServers(Node):
    def __init__(self, context):
        super().__init__('fake_mission_action_servers', context=context)
        group = ReentrantCallbackGroup()
        self.nav_goals = []
        self.nav_yaws = []
        self.spray_missions = []
        self.tree_hints = []
        self.working_ranges = []
        self.nav_server = ActionServer(
            self,
            NavigateToPose,
            '/navigate_to_pose',
            self._execute_nav,
            callback_group=group,
        )
        self.spray_server = ActionServer(
            self,
            ExecuteSpray,
            '/arm/execute_spray',
            self._execute_spray,
            callback_group=group,
        )

    def _execute_nav(self, goal_handle):
        pose = goal_handle.request.pose.pose
        self.nav_goals.append((pose.position.x, pose.position.y))
        self.nav_yaws.append(math.atan2(
            2.0 * pose.orientation.w * pose.orientation.z,
            1.0 - 2.0 * pose.orientation.z ** 2))
        goal_handle.succeed()
        return NavigateToPose.Result()

    def _execute_spray(self, goal_handle):
        request = goal_handle.request
        self.spray_missions.append(request.mission_id)
        self.tree_hints.append((
            request.tree_hint.header.frame_id,
            request.tree_hint.point.x,
            request.tree_hint.point.y,
            request.tree_hint.point.z,
        ))
        self.working_ranges.append(float(request.working_range_m))
        result = ExecuteSpray.Result()
        result.success = True
        result.error_code = ExecuteSpray.Result.OK
        result.message = 'fake spray completed at HOME'
        goal_handle.succeed()
        return result


class _Harness(Node):
    def __init__(self, context):
        super().__init__('fake_mission_harness', context=context)
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.reset_client = self.create_client(Trigger, '/mission/reset')
        self.start_client = self.create_client(Trigger, '/mission/start')
        self.manual_client = self.create_client(
            LoadManualMission, '/mission/load_manual')
        self.status = None
        self.plan = None
        self.create_subscription(
            MissionStatus, '/mission/status', self._on_status, qos)
        self.create_subscription(
            MissionPlan, '/mission/plan', self._on_plan, qos)
        self.create_timer(0.05, self._publish_odom)

    def _on_status(self, message):
        self.status = message

    def _on_plan(self, message):
        self.plan = message

    def _publish_odom(self):
        message = Odometry()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'odom'
        message.child_frame_id = 'base_footprint'
        self.odom_pub.publish(message)

    def load_route(self, mission_id, return_home=False, home=(0.0, 0.0, 0.0)):
        """Build the same explicit Qt/RViz route used by production code."""
        request = LoadManualMission.Request()
        request.header.stamp = self.get_clock().now().to_msg()
        request.header.frame_id = 'map'
        request.mission_id = mission_id
        request.return_home_after_mission = return_home
        request.home_pose.position.x = home[0]
        request.home_pose.position.y = home[1]
        request.home_pose.orientation.z = math.sin(home[2] / 2.0)
        request.home_pose.orientation.w = math.cos(home[2] / 2.0)
        for point_id, dock_x, dock_y, tree_y_m in (
                ('point_01', 3.4, 0.2, 1.8),
                ('point_02', 5.4, -0.2, -1.8)):
            point = ManualMissionPoint()
            point.point_id = point_id
            point.docking_pose.position.x = dock_x
            point.docking_pose.position.y = dock_y
            point.docking_pose.orientation.w = 1.0
            point.tree_x_m = 0.0
            point.tree_y_m = tree_y_m
            point.tree_base_z_m = 0.0
            point.spray_duration = 0.2
            request.points.append(point)
        return self.manual_client.call_async(request)

    def load_manual(self, mission_id, points, return_home=False):
        request = LoadManualMission.Request()
        request.header.stamp = self.get_clock().now().to_msg()
        request.header.frame_id = 'map'
        request.mission_id = mission_id
        request.home_pose.orientation.w = 1.0
        request.return_home_after_mission = return_home
        for point_id, x, y, yaw, tree_x_m, tree_y_m in points:
            point = ManualMissionPoint()
            point.point_id = point_id
            point.docking_pose.position.x = x
            point.docking_pose.position.y = y
            point.docking_pose.orientation.z = math.sin(yaw / 2.0)
            point.docking_pose.orientation.w = math.cos(yaw / 2.0)
            point.spray_duration = 0.2
            point.tree_x_m = tree_x_m
            point.tree_y_m = tree_y_m
            point.tree_base_z_m = 0.0
            request.points.append(point)
        return self.manual_client.call_async(request)


def _spin_until(executor, predicate, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)
        if predicate():
            return True
    return False


def test_three_two_point_fake_closed_loops_complete_in_order():
    context = Context()
    rclpy.init(context=context)
    servers = _FakeServers(context)
    manager = MissionManager(
        context=context,
        parameter_overrides=[
            Parameter('stop_stable_duration_sec', value=0.2),
            Parameter('odom_stale_timeout_sec', value=0.5),
            Parameter('stop_verify_timeout_sec', value=2.0),
            Parameter('nav_goal_timeout_sec', value=3.0),
            Parameter('spray_goal_timeout_sec', value=3.0),
        ],
    )
    harness = _Harness(context)
    executor = SingleThreadedExecutor(context=context)
    for node in (servers, manager, harness):
        executor.add_node(node)

    try:
        assert _spin_until(executor, manager._servers_ready)
        assert _spin_until(executor, harness.reset_client.service_is_ready)

        for run in range(3):
            mission_id = f'fake_closed_loop_{run}'
            load = harness.load_route(mission_id)
            assert _spin_until(executor, load.done)
            assert load.result().success
            start = harness.start_client.call_async(Trigger.Request())
            assert _spin_until(executor, start.done)
            assert start.result().success
            assert _spin_until(
                executor,
                lambda: (
                    harness.status is not None
                    and harness.status.mission_id == mission_id
                    and harness.status.state
                    == MissionStatus.MISSION_COMPLETED
                ),
            )
            if run < 2:
                future = harness.reset_client.call_async(Trigger.Request())
                assert _spin_until(executor, future.done)
                assert future.result().success

        expected_nav = [(3.4, 0.2), (5.4, -0.2)] * 3
        assert len(servers.nav_goals) == len(expected_nav)
        for actual, expected in zip(servers.nav_goals, expected_nav):
            assert math.isclose(actual[0], expected[0], abs_tol=1e-6)
            assert math.isclose(actual[1], expected[1], abs_tol=1e-6)
        assert servers.spray_missions == [
            f'fake_closed_loop_{run}' for run in range(3) for _ in range(2)]
        expected_hints = [
            ('map', 3.0, 2.0, 0.0),
            ('map', 5.0, -2.0, 0.0),
        ] * 3
        for actual, expected in zip(servers.tree_hints, expected_hints):
            assert actual[0] == expected[0]
            assert actual[1:] == pytest.approx(expected[1:])
        assert servers.working_ranges == [0.0] * len(servers.spray_missions)
        assert harness.plan.mission_id == 'fake_closed_loop_2'
        assert [item.point_id for item in harness.plan.points] == [
            'point_01', 'point_02']
        assert math.isclose(
            harness.plan.points[0].docking_pose.position.y,
            0.2,
            abs_tol=1e-6,
        )
    finally:
        for _ in range(5):
            executor.spin_once(timeout_sec=0.02)
        manager._nav_client.destroy()
        manager._spray_client.destroy()
        servers.nav_server.destroy()
        servers.spray_server.destroy()
        harness.destroy_client(harness.reset_client)
        for node in (harness, manager, servers):
            executor.remove_node(node)
        for node in (harness, manager, servers):
            node.destroy_node()
        executor.shutdown(timeout_sec=1.0)
        context.try_shutdown()


def test_optional_return_home_adds_final_nav_goal():
    context = Context()
    rclpy.init(context=context)
    servers = _FakeServers(context)
    manager = MissionManager(
        context=context,
        parameter_overrides=[
            Parameter('return_home_after_mission', value=True),
            Parameter('home_x', value=0.25),
            Parameter('home_y', value=-0.1),
            Parameter('home_yaw', value=0.0),
            Parameter('stop_stable_duration_sec', value=0.1),
            Parameter('odom_stale_timeout_sec', value=0.5),
            Parameter('stop_verify_timeout_sec', value=2.0),
            Parameter('nav_goal_timeout_sec', value=3.0),
            Parameter('spray_goal_timeout_sec', value=3.0),
        ],
    )
    harness = _Harness(context)
    executor = SingleThreadedExecutor(context=context)
    for node in (servers, manager, harness):
        executor.add_node(node)

    try:
        assert _spin_until(executor, manager._servers_ready)
        load = harness.load_route(
            'return_home_loop', return_home=True, home=(0.25, -0.1, 0.0))
        assert _spin_until(executor, load.done)
        assert load.result().success
        start = harness.start_client.call_async(Trigger.Request())
        assert _spin_until(executor, start.done)
        assert start.result().success
        assert _spin_until(
            executor,
            lambda: (
                harness.status is not None
                and harness.status.mission_id == 'return_home_loop'
                and harness.status.state == MissionStatus.MISSION_COMPLETED
            ),
        )
        assert servers.nav_goals == [
            (3.4, 0.2), (5.4, -0.2), (0.25, -0.1)]
        assert servers.spray_missions == ['return_home_loop'] * 2
        assert servers.working_ranges == [0.0, 0.0]
    finally:
        for _ in range(5):
            executor.spin_once(timeout_sec=0.02)
        manager._nav_client.destroy()
        manager._spray_client.destroy()
        servers.nav_server.destroy()
        servers.spray_server.destroy()
        for node in (harness, manager, servers):
            executor.remove_node(node)
            node.destroy_node()
        executor.shutdown(timeout_sec=1.0)
        context.try_shutdown()


def test_manual_mission_preserves_rviz_pose_and_yaw():
    context = Context()
    rclpy.init(context=context)
    servers = _FakeServers(context)
    manager = MissionManager(
        context=context,
        parameter_overrides=[
            Parameter('stop_stable_duration_sec', value=0.1),
            Parameter('odom_stale_timeout_sec', value=0.5),
            Parameter('stop_verify_timeout_sec', value=2.0),
            Parameter('nav_goal_timeout_sec', value=3.0),
            Parameter('spray_goal_timeout_sec', value=3.0),
        ],
    )
    harness = _Harness(context)
    executor = SingleThreadedExecutor(context=context)
    for node in (servers, manager, harness):
        executor.add_node(node)

    try:
        assert _spin_until(executor, manager._servers_ready)
        assert _spin_until(executor, harness.manual_client.service_is_ready)
        assert _spin_until(executor, harness.start_client.service_is_ready)
        load = harness.load_manual(
            'manual_pose_loop',
            [('single_01', 3.2, 0.7, 0.4, 0.0, 1.5)],
        )
        assert _spin_until(executor, load.done)
        assert load.result().success
        start = harness.start_client.call_async(Trigger.Request())
        assert _spin_until(executor, start.done)
        assert start.result().success
        assert _spin_until(
            executor,
            lambda: (
                harness.status is not None
                and harness.status.mission_id == 'manual_pose_loop'
                and harness.status.state == MissionStatus.MISSION_COMPLETED),
        )
        assert servers.nav_goals == [(3.2, 0.7)]
        assert math.isclose(servers.nav_yaws[0], 0.4, abs_tol=1e-6)
        assert servers.spray_missions == ['manual_pose_loop']
        assert servers.working_ranges == [0.0]
        assert servers.tree_hints[0][0] == 'map'
        assert servers.tree_hints[0][1:] == pytest.approx(
            (2.247448, 1.925824, 0.0))
        assert harness.plan.points[0].docking_pose.position.x == 3.2
        assert harness.plan.points[0].docking_pose.position.y == 0.7
    finally:
        for _ in range(5):
            executor.spin_once(timeout_sec=0.02)
        manager._nav_client.destroy()
        manager._spray_client.destroy()
        servers.nav_server.destroy()
        servers.spray_server.destroy()
        for client in (
                harness.reset_client, harness.start_client, harness.manual_client):
            harness.destroy_client(client)
        for node in (harness, manager, servers):
            executor.remove_node(node)
            node.destroy_node()
        executor.shutdown(timeout_sec=1.0)
        context.try_shutdown()
