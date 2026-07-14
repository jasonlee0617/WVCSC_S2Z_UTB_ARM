"""Timed replay of previously recorded UAV mission events."""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from wvcsc_interfaces.msg import DiseaseTreeArray

from .message_factory import mission_message
from .validation import load_and_validate_replay


class ReplayUavGateway(Node):
    def __init__(self, **kwargs):
        super().__init__('replay_uav_gateway', **kwargs)
        self.declare_parameter('config_file', '')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('min_spray_duration', 0.2)
        self.declare_parameter('max_spray_duration', 10.0)
        self.declare_parameter('max_abs_coordinate', 50.0)
        self._config = load_and_validate_replay(
            str(self.get_parameter('config_file').value),
            float(self.get_parameter('confidence_threshold').value),
            float(self.get_parameter('min_spray_duration').value),
            float(self.get_parameter('max_spray_duration').value),
            float(self.get_parameter('max_abs_coordinate').value),
        )
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._publisher = self.create_publisher(
            DiseaseTreeArray, '/uav/disease_trees', qos)
        self._index = 0
        self._started_at = self._now()
        self._timer = self.create_timer(0.02, self._tick)

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _tick(self):
        elapsed = (
            (self._now() - self._started_at)
            * self._config['playback_rate'])
        events = self._config['events']
        while self._index < len(events) and events[self._index]['at_sec'] <= elapsed:
            event = events[self._index]
            message = mission_message(
                event['mission'], self.get_clock().now().to_msg())
            self._publisher.publish(message)
            self.get_logger().info(
                f'[UAV_REPLAY] published mission={message.mission_id} '
                f'targets={len(message.trees)} event={self._index + 1}/{len(events)}')
            self._index += 1
        if self._index < len(events):
            return
        if not self._config['loop']:
            self._timer.cancel()
            return
        self._index = 0
        self._started_at = (
            self._now()
            + self._config['loop_delay_sec'] / self._config['playback_rate'])


def main():
    rclpy.init()
    node = ReplayUavGateway()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
