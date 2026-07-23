"""Launch eye-to-hand calibration for a fixed external camera.

The ArUco marker must be attached to the robot's end-effector.
The camera is fixed in the workspace looking down at the robot.
The gripper remains closed throughout calibration.

Camera must be launched separately beforehand, e.g.:
    ros2 launch realsense2_camera rs_launch.py \\
        camera_namespace:=cam1 camera_name:=cam1 \\
        serial_no:="'<serial>'" enable_infra1:=true enable_infra2:=true infra_rgb:=true

Usage:
    # Calibrate cam1 with master arm (default)
    ros2 launch alicia_m_calibration eye_to_hand_calibration.launch.py

    # Calibrate cam2 with slave arm
    ros2 launch alicia_m_calibration eye_to_hand_calibration.launch.py \\
        camera_topic:=/cam2/cam2/color/image_raw \\
        camera_info_topic:=/cam2/cam2/color/camera_info \\
        output_file:=eye_to_hand_cam2_result.yaml \\
        arm_id:=slave
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # ArUco settings
        DeclareLaunchArgument('aruco_dict', default_value='DICT_4X4_50',
                              description='ArUco dictionary'),
        DeclareLaunchArgument('marker_size', default_value='0.05',
                              description='Marker size in meters'),
        DeclareLaunchArgument('marker_id', default_value='0',
                              description='Target ArUco marker ID'),

        # Camera settings (default: cam1)
        DeclareLaunchArgument('camera_topic',
                              default_value='/cam1/cam1/color/image_raw',
                              description='Camera image topic'),
        DeclareLaunchArgument('camera_info_topic',
                              default_value='/cam1/cam1/color/camera_info',
                              description='Camera info topic'),

        # Robot settings
        DeclareLaunchArgument('base_link', default_value='base_link',
                              description='Robot base frame'),
        DeclareLaunchArgument('end_effector_link', default_value='tool0',
                              description='End effector frame'),

        # Calibration settings
        DeclareLaunchArgument('min_samples', default_value='18',
                              description='Minimum calibration samples'),
        DeclareLaunchArgument('calibration_method', default_value='daniilidis',
                              description='Calibration algorithm'),
        DeclareLaunchArgument('output_file',
                              default_value='eye_to_hand_cam1_result.yaml',
                              description='Output YAML file name'),
        DeclareLaunchArgument('arm_id', default_value='master',
                              description='Arm identifier (master or slave)'),

        # Calibration node
        Node(
            package='alicia_m_calibration',
            executable='eye_to_hand_calibration.py',
            name='eye_to_hand_calibration',
            output='screen',
            parameters=[{
                'aruco_dict': LaunchConfiguration('aruco_dict'),
                'marker_size': LaunchConfiguration('marker_size'),
                'marker_id': LaunchConfiguration('marker_id'),
                'camera_topic': LaunchConfiguration('camera_topic'),
                'camera_info_topic': LaunchConfiguration('camera_info_topic'),
                'base_link': LaunchConfiguration('base_link'),
                'end_effector_link': LaunchConfiguration('end_effector_link'),
                'min_samples': LaunchConfiguration('min_samples'),
                'calibration_method': LaunchConfiguration('calibration_method'),
                'output_file': LaunchConfiguration('output_file'),
                'arm_id': LaunchConfiguration('arm_id'),
            }],
        ),
    ])
