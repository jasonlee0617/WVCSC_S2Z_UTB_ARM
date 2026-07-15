from collections import deque

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


class CameraWatchdog(Node):
    def __init__(self):
        super().__init__('wvcsc_c10_watchdog')
        defaults = {
            'image_topic': '/camera/camera/color/image_rect_raw',
            'camera_info_topic': '/camera/camera/color/camera_info',
            'expected_width': 1280,
            'expected_height': 720,
            'expected_fps': 30.0,
            'stale_timeout_sec': 1.0,
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)
        self._last_image_time = None
        self._last_header_stamp = None
        self._last_info = None
        self._image = None
        self._samples = deque(maxlen=120)
        self._stamp_regressions = deque(maxlen=120)
        self._publisher = self.create_publisher(
            DiagnosticArray, '/diagnostics', 10)
        self.create_subscription(
            Image, str(self.get_parameter('image_topic').value),
            self._on_image, qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo, str(self.get_parameter('camera_info_topic').value),
            self._on_info, qos_profile_sensor_data)
        self.create_timer(1.0, self._publish)

    def _on_image(self, message):
        now = self.get_clock().now()
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        self._stamp_regressions.append(
            self._last_header_stamp is not None
            and stamp <= self._last_header_stamp)
        self._last_header_stamp = stamp
        self._last_image_time = now
        self._image = message
        self._samples.append(now.nanoseconds * 1e-9)

    def _on_info(self, message):
        self._last_info = message

    def _measured_fps(self):
        if len(self._samples) < 2:
            return 0.0
        span = self._samples[-1] - self._samples[0]
        return (len(self._samples) - 1) / span if span > 0.0 else 0.0

    def _publish(self):
        expected_w = int(self.get_parameter('expected_width').value)
        expected_h = int(self.get_parameter('expected_height').value)
        expected_fps = float(self.get_parameter('expected_fps').value)
        stale = float(self.get_parameter('stale_timeout_sec').value)
        status = DiagnosticStatus()
        status.name = 'wvcsc/c10_camera'
        status.hardware_id = 'Synria-C10'
        now = self.get_clock().now()
        stamp_age = (
            now.nanoseconds * 1e-9 - self._last_header_stamp
            if self._last_header_stamp is not None else float('inf'))

        if self._last_image_time is None or (
                now - self._last_image_time).nanoseconds * 1e-9 > stale:
            status.level = DiagnosticStatus.ERROR
            status.message = 'image stream missing or stale'
        elif self._image.width != expected_w or self._image.height != expected_h:
            status.level = DiagnosticStatus.ERROR
            status.message = 'unexpected image resolution'
        elif self._image.encoding not in ('rgb8', 'bgr8'):
            status.level = DiagnosticStatus.ERROR
            status.message = f'unexpected ROS encoding: {self._image.encoding}'
        elif self._last_header_stamp is None or self._last_header_stamp <= 0.0:
            status.level = DiagnosticStatus.ERROR
            status.message = 'image timestamp is missing'
        elif sum(self._stamp_regressions) > 0:
            status.level = DiagnosticStatus.WARN
            status.message = 'image timestamp is not monotonic'
        elif abs(stamp_age) > stale:
            status.level = DiagnosticStatus.WARN
            status.message = 'image timestamp differs from ROS clock'
        elif self._last_info is None:
            status.level = DiagnosticStatus.WARN
            status.message = 'CameraInfo not received'
        elif self._measured_fps() < expected_fps * 0.70:
            status.level = DiagnosticStatus.WARN
            status.message = 'camera frame rate below threshold'
        else:
            status.level = DiagnosticStatus.OK
            status.message = 'camera stream healthy'

        width = self._image.width if self._image is not None else 0
        height = self._image.height if self._image is not None else 0
        encoding = self._image.encoding if self._image is not None else ''
        status.values = [
            KeyValue(key='resolution', value=f'{width}x{height}'),
            KeyValue(key='encoding', value=encoding),
            KeyValue(key='measured_fps', value=f'{self._measured_fps():.2f}'),
            KeyValue(key='expected_fps', value=f'{expected_fps:.2f}'),
            KeyValue(key='stamp_age_sec', value=f'{stamp_age:.3f}'),
            KeyValue(
                key='recent_stamp_regressions',
                value=str(sum(self._stamp_regressions))),
        ]
        message = DiagnosticArray()
        message.header.stamp = now.to_msg()
        message.status = [status]
        self._publisher.publish(message)


def main():
    rclpy.init()
    node = CameraWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
