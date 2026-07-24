# c10_camera.launch.py
# ============================================================================
# Synria C10 相机驱动启动脚本
# ============================================================================
#
# 职责：
# 1. 使用 ROS2 标准的 `usb_cam` 包启动 C10 相机硬件驱动。
# 2. 将 `c10_usb_cam.yaml` 参数注入驱动节点。
# 3. 启动配套的 `camera_watchdog` 监控节点，实时检测相机流状态。
# 4. 实现了硬件掉线后自动重连机制（Respawn）。
#

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 1. 定位本功能包的共享目录，加载 `c10_usb_cam.yaml` 配置文件。
    # 该文件包含了内参文件路径、帧率、分辨率等所有固定配置。
    share = get_package_share_directory('wvcsc_c10_camera')
    config = os.path.join(share, 'config', 'c10_usb_cam.yaml')

    # 2. 声明 `video_device` 启动参数，允许在运行命令时动态指定。
    # 例如：`ros2 launch wvcsc_c10_camera c10_camera.launch.py video_device:=/dev/video2`
    # 本机 usb_cam 对 /dev/v4l/by-id 符号链接解析异常，默认使用 JR0037 index0。
    device = LaunchConfiguration('video_device')

    # 3. 配置相机驱动节点 (`usb_cam`)
    camera = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='c10_driver',                   # 节点名称
        namespace='/camera/color',    # 添加命名空间，生成的话题会是 /camera/color/image_raw
        parameters=[config, {
            'video_device': device,
            'camera_info_url': LaunchConfiguration('camera_info_url'),
        }], # 合并 YAML 配置与动态传入的设备路径
        output='screen',                     # 将驱动日志输出到终端屏幕
        respawn=True,                        # 【核心设计】如果因物理接触导致 USB 掉线，驱动节点崩溃，
                                             # ROS 会尝试自动重启该节点。
        respawn_delay=2.0,                   # 掉线后等待 2 秒再尝试重启，避免高频重连造成系统卡死。
    )

    # 4. 配置相机状态看门狗节点。
    # 该节点独立于驱动运行，专门负责报告相机是否“健康”。
    watchdog = Node(
        package='wvcsc_c10_camera',
        executable='camera_watchdog',
        parameters=[{
            'image_topic': '/camera/color/image_raw',
            'camera_info_topic': '/camera/color/camera_info',
            'expected_width': 640,           # 期望的分辨率，与标定文件一致
            'expected_height': 480,
            'expected_fps': 30.0,            # 期望的帧率，与 YOLO 推理节奏保持一致
        }],
        output='screen',
    )

    # 5. 构建 Launch 描述，并声明可选的视频设备路径参数。
    return LaunchDescription([
        DeclareLaunchArgument(
            'video_device',
            default_value='/dev/video2',
            description='JR0037 C10 V4L2 index0 device.'),
        DeclareLaunchArgument(
            'camera_info_url',
            default_value=(
                'package://wvcsc_c10_camera/config/'
                'c10_intrinsics.yaml'),
            description=(
                'CameraInfo URL. After calibration pass '
                'the calibrated package config file explicitly.')),
        camera,
        watchdog,
    ])
