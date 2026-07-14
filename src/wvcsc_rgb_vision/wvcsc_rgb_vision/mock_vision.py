"""Deterministic RGB target source for interface and Web integration tests."""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from wvcsc_interfaces.msg import MissionStatus, Target2D


class MockVision(Node):
    def __init__(self, **kwargs):
        super().__init__('wvcsc_mock_vision', **kwargs)
        parameters = {
            'image_width': 1280,
            'image_height': 720,
            'error_u': 0.0,
            'error_v': 0.0,
            'confidence': 0.95,
            'publish_rate_hz': 10.0,
        }
        for name, default in parameters.items():
            self.declare_parameter(name, default)
        self._mission_id = ''
        self._tree_id = ''
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            MissionStatus, '/mission/status', self._on_status, latched)
        self._publisher = self.create_publisher(Target2D, '/vision/target', 10)
        rate = float(self.get_parameter('publish_rate_hz').value)
        if rate <= 0.0:
            raise ValueError('publish_rate_hz must be positive')
        self.create_timer(1.0 / rate, self._publish)

    def _on_status(self, message):
        self._mission_id = message.mission_id
        self._tree_id = (
            message.current_tree_id
            if message.state == MissionStatus.ARM_SPRAYING else '')

    def _publish(self):
        if not self._tree_id:
            return
        width = int(self.get_parameter('image_width').value)
        height = int(self.get_parameter('image_height').value)
        message = Target2D()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'camera_color_optical_frame'
        message.mission_id = self._mission_id
        message.tree_id = self._tree_id
        message.valid = True
        message.confidence = float(self.get_parameter('confidence').value)
        message.center_u = width / 2.0 + float(
            self.get_parameter('error_u').value)
        message.center_v = height / 2.0 + float(
            self.get_parameter('error_v').value)
        message.width = 120.0
        message.height = 120.0
        message.image_width = width
        message.image_height = height
        self._publisher.publish(message)


def main():
    rclpy.init()
    node = MockVision()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
