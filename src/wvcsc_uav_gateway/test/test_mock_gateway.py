import os
import time

os.environ['ROS_DOMAIN_ID'] = '81'
os.environ.setdefault('ROS_LOG_DIR', '/tmp/wvcsc_uav_test_logs')

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from wvcsc_interfaces.msg import DiseaseTreeArray

from wvcsc_uav_gateway.mock_uav_gateway import MockUavGateway


def _spin_until(executor, predicate, timeout=3.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)
        if predicate():
            return True
    return False


def test_gateway_latches_two_targets_for_late_subscriber(tmp_path):
    config = tmp_path / 'mock_targets.yaml'
    config.write_text('''
mission:
  mission_id: runtime_test
  frame_id: map
  source_mode: mock
  publish_delay_sec: 0.01
  trees:
    - tree_id: tree_01
      confidence: 0.96
      position: {x: 3.0, y: 2.0, z: 0.0}
      spray_side: left
      spray_duration: 2.0
    - tree_id: tree_02
      confidence: 0.94
      position: {x: 5.0, y: -2.0, z: 0.0}
      spray_side: right
      spray_duration: 2.0
''', encoding='utf-8')

    context = Context()
    rclpy.init(context=context)
    gateway = MockUavGateway(
        context=context,
        parameter_overrides=[
            Parameter('config_file', value=str(config)),
        ],
    )
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(gateway)
    listener = None

    try:
        assert _spin_until(executor, gateway._timer.is_canceled)

        listener = Node('late_uav_mission_listener', context=context)
        messages = []
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        listener.create_subscription(
            DiseaseTreeArray,
            '/uav/disease_trees',
            messages.append,
            qos,
        )
        executor.add_node(listener)

        assert _spin_until(executor, lambda: bool(messages))
        assert messages[-1].mission_id == 'runtime_test'
        assert messages[-1].header.frame_id == 'map'
        assert [tree.tree_id for tree in messages[-1].trees] == [
            'tree_01', 'tree_02']
    finally:
        if listener is not None:
            executor.remove_node(listener)
            listener.destroy_node()
        executor.remove_node(gateway)
        gateway.destroy_node()
        executor.shutdown()
        context.try_shutdown()
