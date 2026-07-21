"""Eye-in-hand calibration: Alicia-M arm + C10 camera on a desk with ArUco marker."""
import os
import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, RegisterEventHandler,
    SetEnvironmentVariable, TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    simulation_share = get_package_share_directory('wvcsc_simulation')
    calibration_share = get_package_share_directory('wvcsc_calibration')
    description_share = get_package_share_directory('wvcsc_description')
    gazebo_share = get_package_share_directory('gazebo_ros')
    moveit_share = get_package_share_directory('alicia_m_moveit_config')
    alicia_dir = os.path.dirname(get_package_share_directory('alicia_m_descriptions'))
    easy_share = get_package_share_directory('easy_handeye2')

    model_path = os.pathsep.join(filter(None, [
        os.path.join(simulation_share, 'models'),
        alicia_dir,
        os.path.dirname(description_share),
        os.environ.get('GAZEBO_MODEL_PATH'),
    ]))

    controllers_file = os.path.join(description_share, 'config', 'ros2_controllers.yaml')
    xacro_file = os.path.join(calibration_share, 'xacro', 'calibration_arm_camera.urdf.xacro')

    # Build URDF to a temp file so spawn_entity.py -file avoids --param injection.
    urdf_path = '/tmp/alicia_calibration.urdf'
    subprocess.run(['xacro', xacro_file, f'gazebo_controllers_file:={controllers_file}',
                    '-o', urdf_path], check=True)
    urdf = open(urdf_path).read()

    srdf_path = os.path.join(moveit_share, 'config', 'alicia_m_v1_1_follower.srdf')
    with open(srdf_path, encoding='utf-8') as f:
        semantic = f.read()
    semantic = semantic.replace('name="alicia_m_v1_1_follower"',
                                'name="alicia_calibration"')

    moveit_controllers = {
        'moveit_controller_manager':
            'moveit_simple_controller_manager/MoveItSimpleControllerManager',
        'moveit_manage_controllers': True,
    }

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, 'launch', 'gazebo.launch.py')),
        launch_arguments={
            'world': os.path.join(simulation_share, 'worlds', 'calibration_table.world'),
            'verbose': 'false',
            'pause': 'false',
        }.items(),
    )

    state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': urdf}, {'use_sim_time': True}],
        output='screen',
    )

    spawn = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-file', urdf_path, '-entity', 'alicia_calibration',
                   '-x', '0', '-y', '0', '-z', '0.76'],
        output='screen',
    )

    move_group = Node(
        package='moveit_ros_move_group', executable='move_group',
        parameters=[{'robot_description': urdf},
                    {'robot_description_semantic': semantic},
                    moveit_controllers, {'use_sim_time': True}],
        output='screen',
    )

    spawner_args = [
        '--controller-manager', '/controller_manager',
        '--controller-manager-timeout', '30.0',
        '--switch-timeout', '30.0',
    ]
    joint_state_controller = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster', *spawner_args], output='screen',
    )
    arm_controller = Node(
        package='controller_manager', executable='spawner',
        arguments=['arm_controller', *spawner_args], output='screen',
    )

    aruco_node = Node(
        package='ros2_aruco', executable='aruco_node',
        parameters=[{
            'marker_size': 0.07, 'aruco_dictionary_id': 'DICT_5X5_250',
            'marker_id': 1,
            'image_topic': '/camera/color/image_raw',
            'camera_info_topic': '/camera/color/camera_info',
            'camera_frame': 'camera_color_optical_frame',
            'use_sim_time': True,
        }],
        output='screen',
    )
    aruco_tf = Node(
        package='wvcsc_calibration', executable='aruco_tf_broadcaster',
        parameters=[{
            'marker_id': 1, 'aruco_markers_topic': '/aruco_markers',
            'parent_frame_id': 'camera_color_optical_frame',
            'child_frame_id': 'aruco_marker',
            'use_sim_time': True,
        }],
        output='screen',
    )
    handeye = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(easy_share, 'launch', 'calibrate.launch.py')),
        launch_arguments={
            'name': 'alicia_c10_eye_in_hand',
            'calibration_type': 'eye_in_hand',
            'robot_base_frame': 'base_link',
            'robot_effector_frame': 'tool0',
            'tracking_base_frame': 'camera_color_optical_frame',
            'tracking_marker_frame': 'aruco_marker',
            'use_sim_time': 'true',
        }.items(),
    )

    start_jsc = RegisterEventHandler(OnProcessExit(
        target_action=spawn,
        on_exit=[TimerAction(period=0.5, actions=[joint_state_controller])],
    ))
    start_arm = RegisterEventHandler(OnProcessExit(
        target_action=joint_state_controller, on_exit=[arm_controller],
    ))
    start_vis = RegisterEventHandler(OnProcessExit(
        target_action=arm_controller,
        on_exit=[TimerAction(period=1.0, actions=[aruco_node, aruco_tf])],
    ))

    return LaunchDescription([
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', model_path),
        gazebo,
        state_publisher,
        spawn,
        move_group,
        start_jsc,
        start_arm,
        start_vis,
        TimerAction(period=8.0, actions=[handeye]),
    ])
