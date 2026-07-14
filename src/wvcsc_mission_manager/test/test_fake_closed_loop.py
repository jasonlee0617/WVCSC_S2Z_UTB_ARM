import math
import os
import time

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
from wvcsc_interfaces.msg import (
    DiseaseTree,
    DiseaseTreeArray,
    MissionPlan,
    MissionStatus,
)

from wvcsc_mission_manager.mission_manager import MissionManager


class _FakeServers(Node):
    def __init__(self, context):
        super().__init__('fake_mission_action_servers', context=context)
        group = ReentrantCallbackGroup()
        self.nav_goals = []
        self.spray_goals = []
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
        goal_handle.succeed()
        return NavigateToPose.Result()

    def _execute_spray(self, goal_handle):
        request = goal_handle.request
        self.spray_goals.append((request.tree_id, request.spray_side))
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
        self.mission_pub = self.create_publisher(
            DiseaseTreeArray, '/uav/disease_trees', qos)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.reset_client = self.create_client(Trigger, '/mission/reset')
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

    def publish_mission(self, mission_id):
        message = DiseaseTreeArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'map'
        message.mission_id = mission_id
        message.source_mode = 'mock'
        for tree_id, x, y, side in (
                ('tree_01', 3.0, 2.0, 'left'),
                ('tree_02', 5.0, -2.0, 'right')):
            tree = DiseaseTree()
            tree.tree_id = tree_id
            tree.position.x = x
            tree.position.y = y
            tree.confidence = 0.95
            tree.spray_side = side
            tree.spray_duration = 0.2
            message.trees.append(tree)
        self.mission_pub.publish(message)


def _spin_until(executor, predicate, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)
        if predicate():
            return True
    return False


def test_three_two_target_fake_closed_loops_complete_in_order():
    context = Context()
    rclpy.init(context=context)
    servers = _FakeServers(context)
    manager = MissionManager(
        context=context,
        parameter_overrides=[
            Parameter('auto_start', value=True),
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
            harness.publish_mission(mission_id)
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

        expected_nav = [(3.0, 0.5), (5.0, -0.5)] * 3
        assert len(servers.nav_goals) == len(expected_nav)
        for actual, expected in zip(servers.nav_goals, expected_nav):
            assert math.isclose(actual[0], expected[0], abs_tol=1e-6)
            assert math.isclose(actual[1], expected[1], abs_tol=1e-6)
        assert servers.spray_goals == [
            ('tree_01', 'left'),
            ('tree_02', 'right'),
        ] * 3
        assert harness.plan.mission_id == 'fake_closed_loop_2'
        assert [item.target.tree_id for item in harness.plan.targets] == [
            'tree_01', 'tree_02']
        assert math.isclose(
            harness.plan.targets[0].docking_pose.position.y,
            0.5,
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
            Parameter('auto_start', value=True),
            Parameter('return_home_after_finish', value=True),
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
        harness.publish_mission('return_home_loop')
        assert _spin_until(
            executor,
            lambda: (
                harness.status is not None
                and harness.status.mission_id == 'return_home_loop'
                and harness.status.state == MissionStatus.MISSION_COMPLETED
            ),
        )
        assert servers.nav_goals == [
            (3.0, 0.5), (5.0, -0.5), (0.25, -0.1)]
        assert servers.spray_goals == [
            ('tree_01', 'left'), ('tree_02', 'right')]
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
