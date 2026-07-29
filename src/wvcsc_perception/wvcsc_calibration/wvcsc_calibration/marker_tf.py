# 中文说明：对选定 ArUco 目标进行时间平均并发布带时间戳 TF 的节点模块。
# 该 TF 供标定/验证消费，不参与视觉伺服或喷洒触发。
"""Publish a temporally averaged selected ArUco pose as timestamped TF."""

from collections import deque
import math
import time

from geometry_msgs.msg import PoseStamped, TransformStamped
import rclpy
from rclpy.node import Node
from ros2_aruco_interfaces.msg import ArucoMarkers
from tf2_ros import TransformBroadcaster


def average_marker_pose(poses):
    """Average marker poses while treating opposite quaternion signs equally.

    A single planar-ArUco PnP estimate is intentionally still published by
    ``ros2_aruco`` for real-time consumers.  Hand-eye sampling happens only
    after the arm has settled, so its TF should instead represent the stable
    window of measurements.  This helper performs no frame conversion: all
    supplied poses are already camera-to-marker transforms.
    """
    if not poses:
        raise ValueError('at least one marker pose is required')
    count = float(len(poses))
    translation = tuple(
        sum(float(pose[0][axis]) for pose in poses) / count
        for axis in range(3))
    reference = tuple(float(value) for value in poses[0][1])
    quaternions = []
    for _position, quaternion in poses:
        values = tuple(float(value) for value in quaternion)
        if sum(left * right for left, right in zip(values, reference)) < 0.0:
            values = tuple(-value for value in values)
        quaternions.append(values)
    average = tuple(
        sum(quaternion[axis] for quaternion in quaternions) / count
        for axis in range(4))
    norm = math.sqrt(sum(value * value for value in average))
    if not math.isfinite(norm) or norm <= 1.0e-12:
        raise ValueError('marker quaternion average is invalid')
    return translation, tuple(value / norm for value in average)


class MarkerTransformPublisher(Node):
    def __init__(self):
        super().__init__('wvcsc_calibration_marker_tf')
        self.declare_parameter('tracking_base_frame', 'camera_color_optical_frame')
        self.declare_parameter('tracking_marker_frame', 'calibration_aruco')
        self.declare_parameter('marker_id', 1)
        self.declare_parameter('aruco_topic', '/aruco_markers')
        self.declare_parameter('log_period_sec', 5.0)
        self.declare_parameter('smoothing_window', 1)
        # The collector publishes one quality-gated, sub-pixel-refined pose
        # immediately before easy_handeye2 takes a sample.  Keeping the TF
        # authority here prevents a second camera->marker broadcaster while
        # avoiding a race between the generic ArUco node and the sample call.
        self.declare_parameter(
            'stable_pose_topic', '/calibration/stable_marker_pose')
        self.declare_parameter('stable_pose_hold_sec', 1.0)
        self._base = str(self.get_parameter('tracking_base_frame').value)
        self._marker = str(self.get_parameter('tracking_marker_frame').value)
        self._marker_id = int(self.get_parameter('marker_id').value)
        smoothing_window = int(self.get_parameter('smoothing_window').value)
        stable_pose_hold_sec = float(
            self.get_parameter('stable_pose_hold_sec').value)
        if not self._base or not self._marker or self._marker_id < 0:
            raise ValueError('tracking frames and non-negative marker_id are required')
        if smoothing_window < 1:
            raise ValueError('smoothing_window must be at least one')
        if (not math.isfinite(stable_pose_hold_sec)
                or stable_pose_hold_sec <= 0.0):
            raise ValueError('stable_pose_hold_sec must be finite and positive')
        self._broadcaster = TransformBroadcaster(self)
        self._published_once = False
        self._last_log = 0.0
        self._poses = deque(maxlen=smoothing_window)
        self._stable_pose_hold_sec = stable_pose_hold_sec
        self._stable_override_until = 0.0
        self.create_subscription(
            ArucoMarkers, str(self.get_parameter('aruco_topic').value),
            self._on_markers, 10)
        self.create_subscription(
            PoseStamped, str(self.get_parameter('stable_pose_topic').value),
            self._on_stable_pose, 10)

    def _broadcast(self, translation, quaternion, stamp):
        """Broadcast the single authoritative camera-to-selected-marker TF."""
        transform = TransformStamped()
        transform.header.stamp = stamp
        if transform.header.stamp.sec == 0 and transform.header.stamp.nanosec == 0:
            transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self._base
        transform.child_frame_id = self._marker
        transform.transform.translation.x = translation[0]
        transform.transform.translation.y = translation[1]
        transform.transform.translation.z = translation[2]
        transform.transform.rotation.x = quaternion[0]
        transform.transform.rotation.y = quaternion[1]
        transform.transform.rotation.z = quaternion[2]
        transform.transform.rotation.w = quaternion[3]
        self._broadcaster.sendTransform(transform)
        if not self._published_once:
            self._published_once = True
            self.get_logger().info(
                f'[HAND_EYE][MARKER_TF_READY] '
                f'{self._base}->{self._marker}')

    def _on_stable_pose(self, message):
        """Temporarily prefer the collector's quality-gated sample pose.

        ``ros2_aruco`` keeps supplying raw frames for visual diagnostics and
        readiness.  The easy-handeye2 sample itself must use the exact stable
        window which passed the collector's corner, range and motion checks.
        """
        if message.header.frame_id and message.header.frame_id != self._base:
            self.get_logger().warn(
                'ignoring stable marker pose with unexpected frame '
                f'{message.header.frame_id!r}')
            return
        pose = message.pose
        translation = (pose.position.x, pose.position.y, pose.position.z)
        quaternion = (pose.orientation.x, pose.orientation.y,
                      pose.orientation.z, pose.orientation.w)
        if not all(math.isfinite(float(value)) for value in translation):
            self.get_logger().warn(
                'ignoring stable marker pose with non-finite translation')
            return
        try:
            _translation, quaternion = average_marker_pose(
                ((translation, quaternion),))
        except ValueError:
            self.get_logger().warn('ignoring invalid stable marker pose')
            return
        self._poses.clear()
        self._stable_override_until = (
            time.monotonic() + self._stable_pose_hold_sec)
        self._broadcast(translation, quaternion, message.header.stamp)

    def _on_markers(self, message):
        if time.monotonic() < self._stable_override_until:
            return
        try:
            index = list(message.marker_ids).index(self._marker_id)
            pose = message.poses[index]
        except (ValueError, IndexError):
            return
        self._poses.append((
            (pose.position.x, pose.position.y, pose.position.z),
            (pose.orientation.x, pose.orientation.y,
             pose.orientation.z, pose.orientation.w)))
        translation, quaternion = average_marker_pose(self._poses)
        self._broadcast(translation, quaternion, message.header.stamp)
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
