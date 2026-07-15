import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('wvcsc_c10_camera')
    config = os.path.join(share, 'config', 'c10_usb_cam.yaml')
    device = LaunchConfiguration('video_device')

    camera = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='c10_driver',
        namespace='/camera/camera/color',
        parameters=[config, {'video_device': device}],
        remappings=[('image_raw', 'image_rect_raw')],
        output='screen',
        respawn=True,
        respawn_delay=2.0,
    )
    watchdog = Node(
        package='wvcsc_c10_camera',
        executable='camera_watchdog',
        parameters=[{
            'image_topic': '/camera/camera/color/image_rect_raw',
            'camera_info_topic': '/camera/camera/color/camera_info',
            'expected_width': 1280,
            'expected_height': 720,
            'expected_fps': 30.0,
        }],
        output='screen',
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'video_device',
            default_value='/dev/v4l/by-id/usb-Synria_C10-video-index0',
            description='Use the real persistent /dev/v4l/by-id path.'),
        camera,
        watchdog,
    ])
