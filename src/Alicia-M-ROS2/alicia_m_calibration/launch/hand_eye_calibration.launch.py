"""Launch file for hand-eye calibration with Alicia-M robot and RealSense D405 camera."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for hand-eye calibration."""
    
    # 声明启动参数
    aruco_dict_arg = DeclareLaunchArgument(
        'aruco_dict',
        default_value='DICT_4X4_50',
        description='ArUco dictionary type (e.g., DICT_4X4_50, DICT_5X5_100)'
    )
    
    marker_size_arg = DeclareLaunchArgument(
        'marker_size',
        default_value='0.05',
        description='ArUco marker size in meters (5cm default)'
    )
    
    marker_id_arg = DeclareLaunchArgument(
        'marker_id',
        default_value='0',
        description='Target ArUco marker ID'
    )
    
    # RealSense D405 相机话题
    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic',
        default_value='/camera/camera/color/image_rect_raw',
        description='Camera image topic (RealSense D405)'
    )
    
    camera_info_topic_arg = DeclareLaunchArgument(
        'camera_info_topic',
        default_value='/camera/camera/color/camera_info',
        description='Camera info topic (RealSense D405)'
    )
    
    base_link_arg = DeclareLaunchArgument(
        'base_link',
        default_value='base_link',
        description='Robot base link name'
    )
    
    # Alicia-M 使用 tool0 作为末端坐标系
    end_effector_link_arg = DeclareLaunchArgument(
        'end_effector_link',
        default_value='tool0',
        description='Robot end effector link name'
    )
    
    min_samples_arg = DeclareLaunchArgument(
        'min_samples',
        default_value='15',
        description='Minimum number of calibration samples'
    )
    
    calibration_method_arg = DeclareLaunchArgument(
        'calibration_method',
        default_value='daniilidis',
        description='Hand-eye calibration method (tsai, park, horaud, daniilidis, andreff)'
    )
    
    output_file_arg = DeclareLaunchArgument(
        'output_file',
        default_value='hand_eye_calibration_result.yaml',
        description='Output file name for calibration result'
    )
    
    # 手眼标定节点
    calibration_node = Node(
        package='alicia_m_calibration',
        executable='hand_eye_calibration.py',
        name='hand_eye_calibration',
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
        }],
    )
    
    return LaunchDescription([
        # 日志信息
        LogInfo(msg='========================================'),
        LogInfo(msg='Alicia-M Hand-Eye Calibration (Eye-in-Hand)'),
        LogInfo(msg='========================================'),
        LogInfo(msg='Prerequisites:'),
        LogInfo(msg='  1. Start MoveIt: ros2 launch alicia_m_bringup moveit_hardware.launch.py'),
        LogInfo(msg='  2. Start Camera: ros2 launch realsense2_camera rs_launch.py'),
        LogInfo(msg='========================================'),
        LogInfo(msg='TF Tree: tool0 -> camera_link -> camera_color_optical_frame -> aruco_marker'),
        LogInfo(msg='========================================'),
        
        # 参数声明
        aruco_dict_arg,
        marker_size_arg,
        marker_id_arg,
        camera_topic_arg,
        camera_info_topic_arg,
        base_link_arg,
        end_effector_link_arg,
        min_samples_arg,
        calibration_method_arg,
        output_file_arg,
        
        # 节点
        calibration_node,
    ])
