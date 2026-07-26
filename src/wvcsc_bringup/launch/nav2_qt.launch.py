from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_frame = LaunchConfiguration('map_frame')
    base_frame = LaunchConfiguration('base_frame')
    goal_pose_topic = LaunchConfiguration('goal_pose_topic')
    require_global_relocalization_service = LaunchConfiguration(
        'require_global_relocalization_service')
    show_sim_spray_status = LaunchConfiguration('show_sim_spray_status')
    simulation_parking_clearance_check = LaunchConfiguration(
        'simulation_parking_clearance_check')
    observation_mode = LaunchConfiguration('observation_mode')
    ik_recording_range_min_m = LaunchConfiguration('ik_recording_range_min_m')
    ik_recording_range_max_m = LaunchConfiguration('ik_recording_range_max_m')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('base_frame', default_value='base_footprint'),
        DeclareLaunchArgument('goal_pose_topic', default_value='/manual_goal_pose'),
        DeclareLaunchArgument(
            'require_global_relocalization_service', default_value='true'),
        DeclareLaunchArgument('show_sim_spray_status', default_value='false'),
        DeclareLaunchArgument(
            'simulation_parking_clearance_check', default_value='false'),
        DeclareLaunchArgument('observation_mode', default_value='joint_presets'),
        DeclareLaunchArgument('ik_recording_range_min_m', default_value='0.85'),
        DeclareLaunchArgument('ik_recording_range_max_m', default_value='1.45'),
        Node(
            package='wvcsc_bringup',
            executable='nav2_qt.py',
            parameters=[{
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'map_frame': map_frame,
                'base_frame': base_frame,
                'goal_pose_topic': goal_pose_topic,
                'require_global_relocalization_service': ParameterValue(
                    require_global_relocalization_service, value_type=bool),
                'show_sim_spray_status': ParameterValue(
                    show_sim_spray_status, value_type=bool),
                'simulation_parking_clearance_check': ParameterValue(
                    simulation_parking_clearance_check, value_type=bool),
                'observation_mode': observation_mode,
                'ik_recording_range_min_m': ParameterValue(
                    ik_recording_range_min_m, value_type=float),
                'ik_recording_range_max_m': ParameterValue(
                    ik_recording_range_max_m, value_type=float),
            }],
            output='screen',
        ),
    ])
