"""Deterministic two-stage perception source for simulation regression tests."""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from wvcsc_interfaces.msg import MissionStatus, Target2D


def _detection(header, target_id, class_name, confidence, center_u, center_v,
               width, height):
    message = Detection2D()
    message.header = header
    message.id = target_id
    message.bbox.center.position.x = center_u
    message.bbox.center.position.y = center_v
    message.bbox.size_x = width
    message.bbox.size_y = height
    hypothesis = ObjectHypothesisWithPose()
    hypothesis.hypothesis.class_id = class_name
    hypothesis.hypothesis.score = confidence
    message.results = [hypothesis]
    return message


class MockVision(Node):
    def __init__(self):
        super().__init__('wvcsc_mock_vision')
        for name, default in {
                'image_width': 1280,
                'image_height': 720,
                'error_u': 0.0,
                'error_v': 0.0,
                'confidence': 0.95,
                'publish_diseased_fruit': True,
                'publish_rate_hz': 10.0,
        }.items():
            self.declare_parameter(name, default)
        self._mission_id = ''
        self._tree_id = ''
        self._selected_target_id = ''
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(MissionStatus, '/mission/status', self._on_status, latched)
        self.create_subscription(String, '/vision/selected_target_id', self._on_selected, 10)
        self._tree_pub = self.create_publisher(Detection2DArray, '/vision/tree_detections', 10)
        self._fruit_pub = self.create_publisher(Detection2DArray, '/vision/fruit_detections', 10)
        self._target_pub = self.create_publisher(Target2D, '/vision/target', 10)
        rate = float(self.get_parameter('publish_rate_hz').value)
        if rate <= 0.0:
            raise ValueError('publish_rate_hz must be positive')
        self.create_timer(1.0 / rate, self._publish)

    def _on_status(self, message):
        active = message.state == MissionStatus.ARM_SPRAYING
        self._mission_id = message.mission_id if active else ''
        self._tree_id = message.current_tree_id if active else ''

    def _on_selected(self, message):
        self._selected_target_id = message.data.strip()

    def _publish(self):
        if not self._tree_id:
            return
        width = int(self.get_parameter('image_width').value)
        height = int(self.get_parameter('image_height').value)
        confidence = float(self.get_parameter('confidence').value)
        header = Target2D().header
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'camera_color_optical_frame'
        tree = Detection2DArray()
        tree.header = header
        tree.detections = [_detection(
            header, 'tree', 'tree', confidence, width / 2.0, height / 2.0,
            width * 0.7, height * 0.8)]
        fruit = Detection2DArray()
        fruit.header = header
        if self.get_parameter('publish_diseased_fruit').value:
            fruit.detections = [_detection(
                header, 'mock-fruit-1', 'diseased_fruit', confidence,
                width / 2.0, height / 2.0, 120.0, 120.0)]
        self._tree_pub.publish(tree)
        self._fruit_pub.publish(fruit)
        if self._selected_target_id != 'mock-fruit-1':
            return
        target = Target2D()
        target.header = header
        target.mission_id = self._mission_id
        target.tree_id = self._tree_id
        target.target_id = 'mock-fruit-1'
        target.valid = True
        target.confidence = confidence
        target.center_u = width / 2.0 + float(self.get_parameter('error_u').value)
        target.center_v = height / 2.0 + float(self.get_parameter('error_v').value)
        target.width = 120.0
        target.height = 120.0
        target.image_width = width
        target.image_height = height
        self._target_pub.publish(target)


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
