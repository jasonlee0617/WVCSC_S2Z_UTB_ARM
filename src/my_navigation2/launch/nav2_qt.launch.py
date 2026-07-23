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
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('base_frame', default_value='base_footprint'),
        DeclareLaunchArgument('goal_pose_topic', default_value='/manual_goal_pose'),
        Node(
            package='my_navigation2',
            executable='nav2_qt.py',
            parameters=[{
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'map_frame': map_frame,
                'base_frame': base_frame,
                'goal_pose_topic': goal_pose_topic,
            }],
            output='screen',
        ),
    ])
