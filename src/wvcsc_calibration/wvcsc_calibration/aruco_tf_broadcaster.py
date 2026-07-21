#!/usr/bin/env python3
"""将 ros2_aruco 检测消息转换为 camera -> aruco_marker TF。"""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from ros2_aruco_interfaces.msg import ArucoMarkers
from tf2_ros import TransformBroadcaster


class ArucoTfBroadcaster(Node):

    def __init__(self) -> None:
        super().__init__('aruco_tf_broadcaster')

        self.declare_parameter('marker_id', 1)
        self.declare_parameter(
            'aruco_markers_topic',
            '/aruco_markers',
        )
        self.declare_parameter(
            'parent_frame_id',
            'camera_color_optical_frame',
        )
        self.declare_parameter(
            'child_frame_id',
            'aruco_marker',
        )

        self.marker_id = int(
            self.get_parameter('marker_id').value
        )
        self.topic = str(
            self.get_parameter('aruco_markers_topic').value
        )
        self.parent_frame_id = str(
            self.get_parameter('parent_frame_id').value
        )
        self.child_frame_id = str(
            self.get_parameter('child_frame_id').value
        )

        self.tf_broadcaster = TransformBroadcaster(self)

        self.subscription = self.create_subscription(
            ArucoMarkers,
            self.topic,
            self.markers_callback,
            10,
        )

        self.last_warning_ns = 0

        self.get_logger().info(
            f'Waiting for marker id={self.marker_id} on {self.topic}; '
            f'publishing {self.parent_frame_id} -> '
            f'{self.child_frame_id}'
        )

    def warn_throttled(
        self,
        message: str,
        period_sec: float = 2.0,
    ) -> None:
        now_ns = self.get_clock().now().nanoseconds

        if now_ns - self.last_warning_ns >= int(period_sec * 1e9):
            self.get_logger().warn(message)
            self.last_warning_ns = now_ns

    def markers_callback(
        self,
        msg: ArucoMarkers,
    ) -> None:
        marker_ids = list(msg.marker_ids)

        if self.marker_id not in marker_ids:
            self.warn_throttled(
                f'ArUco id={self.marker_id} is not currently detected'
            )
            return

        marker_index = marker_ids.index(self.marker_id)

        if marker_index >= len(msg.poses):
            self.warn_throttled(
                'Invalid ArucoMarkers message: '
                'marker_ids and poses lengths differ'
            )
            return

        pose = msg.poses[marker_index]

        transform = TransformStamped()

        # 必须使用图像检测消息时间戳，
        # easy_handeye2 会查询历史时刻的 TF。
        transform.header.stamp = msg.header.stamp
        transform.header.frame_id = (
            self.parent_frame_id or msg.header.frame_id
        )
        transform.child_frame_id = self.child_frame_id

        transform.transform.translation.x = pose.position.x
        transform.transform.translation.y = pose.position.y
        transform.transform.translation.z = pose.position.z

        transform.transform.rotation = pose.orientation

        self.tf_broadcaster.sendTransform(transform)


def main(args=None) -> None:
    rclpy.init(args=args)

    node = ArucoTfBroadcaster()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
