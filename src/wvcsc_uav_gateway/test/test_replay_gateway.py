import os
import time

os.environ['ROS_DOMAIN_ID'] = '81'
os.environ.setdefault('ROS_LOG_DIR', '/tmp/wvcsc_uav_replay_test_logs')

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from wvcsc_interfaces.msg import DiseaseTreeArray

from wvcsc_uav_gateway.replay_uav_gateway import ReplayUavGateway


def _spin_until(executor, predicate, timeout=3.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.02)
        if predicate():
            return True
    return False


def test_replay_publishes_events_in_recorded_order(tmp_path):
    config = tmp_path / 'replay.yaml'
    config.write_text('''
replay:
  playback_rate: 10.0
  loop: false
  events:
    - at_sec: 0.1
      mission:
        mission_id: replay_1
        frame_id: map
        source_mode: replay
        trees:
          - tree_id: tree_1
            confidence: 0.9
            position: {x: 3.0, y: 2.0}
            spray_side: left
            spray_duration: 0.2
    - at_sec: 0.5
      mission:
        mission_id: replay_2
        frame_id: map
        source_mode: replay
        trees:
          - tree_id: tree_2
            confidence: 0.9
            position: {x: 5.0, y: -2.0}
            spray_side: right
            spray_duration: 0.2
''', encoding='utf-8')
    context = Context()
    rclpy.init(context=context)
    gateway = ReplayUavGateway(
        context=context,
        parameter_overrides=[Parameter('config_file', value=str(config))])
    listener = Node('replay_uav_listener', context=context)
    messages = []
    qos = QoSProfile(
        depth=1, reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL)
    listener.create_subscription(
        DiseaseTreeArray, '/uav/disease_trees', messages.append, qos)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(gateway)
    executor.add_node(listener)
    try:
        assert _spin_until(executor, lambda: len(messages) == 2)
        assert [message.mission_id for message in messages] == [
            'replay_1', 'replay_2']
        assert gateway._timer.is_canceled()
    finally:
        for node in (listener, gateway):
            executor.remove_node(node)
            node.destroy_node()
        executor.shutdown()
        context.try_shutdown()
