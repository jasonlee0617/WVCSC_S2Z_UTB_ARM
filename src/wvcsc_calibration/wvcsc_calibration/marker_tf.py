"""Publish the selected ros2_aruco marker pose as a timestamped TF."""

import time

from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.node import Node
from ros2_aruco_interfaces.msg import ArucoMarkers
from tf2_ros import TransformBroadcaster


class MarkerTransformPublisher(Node):
    def __init__(self):
        super().__init__('wvcsc_calibration_marker_tf')
        self.declare_parameter('tracking_base_frame', 'camera_color_optical_frame')
        self.declare_parameter('tracking_marker_frame', 'calibration_aruco')
        self.declare_parameter('marker_id', 1)
        self.declare_parameter('aruco_topic', '/aruco_markers')
        self.declare_parameter('log_period_sec', 5.0)
        self._base = str(self.get_parameter('tracking_base_frame').value)
        self._marker = str(self.get_parameter('tracking_marker_frame').value)
        self._marker_id = int(self.get_parameter('marker_id').value)
        if not self._base or not self._marker or self._marker_id < 0:
            raise ValueError('tracking frames and non-negative marker_id are required')
        self._broadcaster = TransformBroadcaster(self)
        self._last_log = 0.0
        self.create_subscription(
            ArucoMarkers, str(self.get_parameter('aruco_topic').value),
            self._on_markers, 10)

    def _on_markers(self, message):
        try:
            index = list(message.marker_ids).index(self._marker_id)
            pose = message.poses[index]
        except (ValueError, IndexError):
            return
        transform = TransformStamped()
        transform.header.stamp = message.header.stamp
        if transform.header.stamp.sec == 0 and transform.header.stamp.nanosec == 0:
            transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self._base
        transform.child_frame_id = self._marker
        transform.transform.translation.x = pose.position.x
        transform.transform.translation.y = pose.position.y
        transform.transform.translation.z = pose.position.z
        transform.transform.rotation = pose.orientation
        self._broadcaster.sendTransform(transform)
        now = time.monotonic()
        period = float(self.get_parameter('log_period_sec').value)
        if period > 0.0 and now - self._last_log >= period:
            self._last_log = now
            self.get_logger().info(
                f'marker id={self._marker_id} TF {self._base}->{self._marker}')


def main():
    rclpy.init()
    node = MarkerTransformPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
