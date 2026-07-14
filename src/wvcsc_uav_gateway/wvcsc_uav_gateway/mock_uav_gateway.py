import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from wvcsc_interfaces.msg import DiseaseTreeArray

from .message_factory import mission_message
from .validation import load_and_validate


class MockUavGateway(Node):
    def __init__(self, **kwargs):
        super().__init__('mock_uav_gateway', **kwargs)
        self.declare_parameter('config_file', '')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('min_spray_duration', 0.2)
        self.declare_parameter('max_spray_duration', 10.0)
        self.declare_parameter('max_abs_coordinate', 50.0)
        config = load_and_validate(
            str(self.get_parameter('config_file').value),
            float(self.get_parameter('confidence_threshold').value),
            float(self.get_parameter('min_spray_duration').value),
            float(self.get_parameter('max_spray_duration').value),
            float(self.get_parameter('max_abs_coordinate').value),
        )
        self._config = config
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._publisher = self.create_publisher(
            DiseaseTreeArray, '/uav/disease_trees', qos)
        self._timer = self.create_timer(
            max(0.001, config['publish_delay_sec']), self._publish_once)

    def _publish_once(self):
        self._timer.cancel()
        message = mission_message(
            self._config, self.get_clock().now().to_msg())
        self._publisher.publish(message)
        self.get_logger().info(
            f"[UAV_GATEWAY] published mission={message.mission_id} "
            f"targets={len(message.trees)} frame={message.header.frame_id}")


def main():
    rclpy.init()
    node = MockUavGateway()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
