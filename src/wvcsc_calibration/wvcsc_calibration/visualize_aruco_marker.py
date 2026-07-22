#!/usr/bin/env python3
"""Publish a C10 image with ArUco marker overlay for calibration debugging."""

import math

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image


class ArucoOverlay(Node):
    def __init__(self):
        super().__init__('wvcsc_aruco_overlay')
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')
        self.declare_parameter('output_topic', '/calibration/aruco_debug_image')
        self.declare_parameter('marker_size_m', 0.070)
        self.declare_parameter('aruco_dictionary_id', 'DICT_5X5_250')
        self.declare_parameter('marker_id', 1)

        dictionary_name = str(self.get_parameter('aruco_dictionary_id').value)
        if not hasattr(cv2, 'aruco') or not hasattr(cv2.aruco, dictionary_name):
            raise RuntimeError(
                f'OpenCV ArUco dictionary is unavailable: {dictionary_name}')
        dictionary_id = getattr(cv2.aruco, dictionary_name)
        self._dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
        self._detector_parameters = (
            cv2.aruco.DetectorParameters()
            if hasattr(cv2.aruco, 'DetectorParameters')
            else cv2.aruco.DetectorParameters_create())
        self._bridge = CvBridge()
        self._camera_matrix = None
        self._distortion = None
        sensor_qos = QoSProfile(
            depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            CameraInfo,
            str(self.get_parameter('camera_info_topic').value),
            self._on_camera_info, sensor_qos)
        self.create_subscription(
            Image, str(self.get_parameter('image_topic').value),
            self._on_image, sensor_qos)
        self._publisher = self.create_publisher(
            Image, str(self.get_parameter('output_topic').value), 10)
        self.get_logger().info(
            'ArUco debug image ready: '
            f'image={self.get_parameter("image_topic").value} '
            f'camera_info={self.get_parameter("camera_info_topic").value} '
            f'output={self.get_parameter("output_topic").value}')

    def _on_camera_info(self, message):
        if message.width <= 0 or message.height <= 0:
            return
        camera = np.asarray(message.k, dtype=float).reshape(3, 3)
        if camera[0, 0] <= 0.0 or camera[1, 1] <= 0.0:
            return
        self._camera_matrix = camera
        self._distortion = np.asarray(
            message.d if message.d else [0.0] * 5, dtype=float)

    def _on_image(self, message):
        try:
            image = self._bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
        except Exception as error:
            self.get_logger().warn(f'failed to convert camera image: {error}')
            return
        corners, ids, _rejected = cv2.aruco.detectMarkers(
            image, self._dictionary, parameters=self._detector_parameters)
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(image, corners, ids)
            self._draw_pose_details(image, corners, ids)
        output = self._bridge.cv2_to_imgmsg(image, encoding='bgr8')
        output.header = message.header
        self._publisher.publish(output)

    def _draw_pose_details(self, image, corners, ids):
        if self._camera_matrix is None or self._distortion is None:
            return
        marker_size = float(self.get_parameter('marker_size_m').value)
        target_id = int(self.get_parameter('marker_id').value)
        rotations, translations, _objects = cv2.aruco.estimatePoseSingleMarkers(
            corners, marker_size, self._camera_matrix, self._distortion)
        height, width = image.shape[:2]
        center = (width * 0.5, height * 0.5)
        cv2.drawMarker(
            image, (int(center[0]), int(center[1])), (255, 255, 0),
            markerType=cv2.MARKER_CROSS, markerSize=16, thickness=1)
        for index, marker in enumerate(np.asarray(ids).reshape(-1)):
            if int(marker) != target_id:
                continue
            points = np.asarray(corners[index], dtype=float).reshape(4, 2)
            side_px = float(np.mean([
                np.linalg.norm(points[(side + 1) % 4] - points[side])
                for side in range(4)
            ]))
            marker_center = np.mean(points, axis=0)
            error_u = marker_center[0] - center[0]
            error_v = marker_center[1] - center[1]
            depth = float(np.asarray(translations[index]).reshape(3)[2])
            cv2.drawFrameAxes(
                image, self._camera_matrix, self._distortion,
                rotations[index], translations[index], marker_size * 0.5)
            text = (
                f'id={int(marker)} du={error_u:+.1f}px dv={error_v:+.1f}px '
                f'z={depth:.3f}m side={side_px:.1f}px')
            origin = (int(marker_center[0]) + 8, int(marker_center[1]) - 8)
            cv2.putText(
                image, text, origin, cv2.FONT_HERSHEY_SIMPLEX,
                0.45, (0, 255, 255), 1, cv2.LINE_AA)
            if not math.isfinite(depth):
                self.get_logger().warn('non-finite ArUco depth in debug image')


def main(args=None):
    rclpy.init(args=args)
    node = ArucoOverlay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
