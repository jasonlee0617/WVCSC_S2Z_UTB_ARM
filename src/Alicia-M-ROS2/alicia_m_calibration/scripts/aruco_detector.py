#!/usr/bin/env python3
"""
ArUco 标记检测节点 (Alicia-M)

功能:
- 订阅相机图像和相机信息
- 检测 ArUco 标记
- 发布标记位姿的 TF (camera_color_optical_frame -> aruco_marker_frame)
- 可视化检测结果

TF树连接: tool0 -> camera_link -> ... -> camera_color_optical_frame -> aruco_marker_frame
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np
import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from scipy.spatial.transform import Rotation as R


class ArucoDetector(Node):
    """ArUco 标记检测节点"""
    
    def __init__(self):
        super().__init__('aruco_detector')
        
        # 声明参数
        self.declare_parameter('aruco_dict', 'DICT_4X4_50')
        self.declare_parameter('marker_size', 0.05)
        self.declare_parameter('marker_id', 0)
        self.declare_parameter('camera_topic', '/camera/camera/color/image_rect_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('marker_frame', 'aruco_marker_frame')
        self.declare_parameter('show_image', True)
        
        # 获取参数
        self.aruco_dict_name = self.get_parameter('aruco_dict').value
        self.marker_size = self.get_parameter('marker_size').value
        self.marker_id = self.get_parameter('marker_id').value
        self.camera_topic = self.get_parameter('camera_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.marker_frame = self.get_parameter('marker_frame').value
        self.show_image = self.get_parameter('show_image').value
        
        # 初始化 ArUco 检测器
        self._init_aruco_detector()
        
        # 初始化 CV Bridge
        self.bridge = CvBridge()
        
        # 相机内参
        self.camera_matrix = None
        self.dist_coeffs = None
        self.camera_info_received = False
        
        # TF 广播器
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # QoS 配置
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # 订阅相机话题
        self.image_sub = self.create_subscription(
            Image,
            self.camera_topic,
            self._image_callback,
            qos_profile
        )
        
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self._camera_info_callback,
            10
        )
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('ArUco 标记检测节点已启动 (Alicia-M)')
        self.get_logger().info(f'ArUco 字典: {self.aruco_dict_name}')
        self.get_logger().info(f'标记大小: {self.marker_size * 100:.1f} cm')
        self.get_logger().info(f'目标标记ID: {self.marker_id}')
        self.get_logger().info(f'相机坐标系: {self.camera_frame}')
        self.get_logger().info(f'标记坐标系: {self.marker_frame}')
        self.get_logger().info('=' * 60)
    
    def _init_aruco_detector(self):
        """初始化 ArUco 检测器"""
        aruco_dict_map = {
            'DICT_4X4_50': cv2.aruco.DICT_4X4_50,
            'DICT_4X4_100': cv2.aruco.DICT_4X4_100,
            'DICT_4X4_250': cv2.aruco.DICT_4X4_250,
            'DICT_4X4_1000': cv2.aruco.DICT_4X4_1000,
            'DICT_5X5_50': cv2.aruco.DICT_5X5_50,
            'DICT_5X5_100': cv2.aruco.DICT_5X5_100,
            'DICT_5X5_250': cv2.aruco.DICT_5X5_250,
            'DICT_5X5_1000': cv2.aruco.DICT_5X5_1000,
            'DICT_6X6_50': cv2.aruco.DICT_6X6_50,
            'DICT_6X6_100': cv2.aruco.DICT_6X6_100,
            'DICT_6X6_250': cv2.aruco.DICT_6X6_250,
            'DICT_6X6_1000': cv2.aruco.DICT_6X6_1000,
            'DICT_7X7_50': cv2.aruco.DICT_7X7_50,
            'DICT_7X7_100': cv2.aruco.DICT_7X7_100,
            'DICT_7X7_250': cv2.aruco.DICT_7X7_250,
            'DICT_7X7_1000': cv2.aruco.DICT_7X7_1000,
        }
        
        if self.aruco_dict_name not in aruco_dict_map:
            self.get_logger().warn(f'未知的 ArUco 字典: {self.aruco_dict_name}, 使用默认 DICT_4X4_50')
            self.aruco_dict_name = 'DICT_4X4_50'
        
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_map[self.aruco_dict_name])
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
    
    def _camera_info_callback(self, msg):
        """相机信息回调函数"""
        if not self.camera_info_received:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.camera_info_received = True
            self.get_logger().info('相机内参已接收')
    
    def _image_callback(self, msg):
        """图像回调函数"""
        if not self.camera_info_received:
            return
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            corners, ids, rejected = self.aruco_detector.detectMarkers(gray)
            
            marker_detected = False
            if ids is not None and len(ids) > 0:
                for i, marker_id in enumerate(ids.flatten()):
                    if marker_id == self.marker_id:
                        obj_points = np.array([
                            [-self.marker_size/2, self.marker_size/2, 0],
                            [self.marker_size/2, self.marker_size/2, 0],
                            [self.marker_size/2, -self.marker_size/2, 0],
                            [-self.marker_size/2, -self.marker_size/2, 0]
                        ], dtype=np.float32)
                        
                        success, rvec, tvec = cv2.solvePnP(
                            obj_points, corners[i][0], self.camera_matrix, self.dist_coeffs
                        )
                        
                        if success:
                            self._publish_tf(rvec.flatten(), tvec.flatten(), msg.header.stamp)
                            marker_detected = True
                            
                            if self.show_image:
                                cv2.drawFrameAxes(cv_image, self.camera_matrix, self.dist_coeffs,
                                                 rvec, tvec, self.marker_size * 0.5)
                                
                                distance = np.linalg.norm(tvec)
                                cv2.putText(cv_image, f'Distance: {distance:.3f}m', 
                                           (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                        break
            
            if self.show_image:
                if ids is not None:
                    cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
                
                status_color = (0, 255, 0) if marker_detected else (0, 0, 255)
                status_text = f'Marker {self.marker_id}: {"Detected" if marker_detected else "Not Found"}'
                cv2.putText(cv_image, status_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
                
                cv2.imshow('Alicia-M ArUco Detection', cv_image)
                cv2.waitKey(1)
                
        except Exception as e:
            self.get_logger().error(f'图像处理错误: {e}')
    
    def _publish_tf(self, rvec, tvec, timestamp):
        """发布 TF 变换"""
        rotation = R.from_rotvec(rvec)
        quaternion = rotation.as_quat()
        
        t = TransformStamped()
        t.header.stamp = timestamp
        t.header.frame_id = self.camera_frame
        t.child_frame_id = self.marker_frame
        
        t.transform.translation.x = float(tvec[0])
        t.transform.translation.y = float(tvec[1])
        t.transform.translation.z = float(tvec[2])
        
        t.transform.rotation.x = float(quaternion[0])
        t.transform.rotation.y = float(quaternion[1])
        t.transform.rotation.z = float(quaternion[2])
        t.transform.rotation.w = float(quaternion[3])
        
        self.tf_broadcaster.sendTransform(t)
    
    def shutdown(self):
        """关闭节点"""
        if self.show_image:
            cv2.destroyAllWindows()


def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    node = ArucoDetector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
