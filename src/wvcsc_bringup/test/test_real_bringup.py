"""Static architecture contracts for the real WVCSC launch boundary.

These tests intentionally inspect launch sources instead of starting hardware.
They catch accidental reintroduction of Cartographer into localization mode,
duplicate hardware stacks, and drift from the field-validated Nav2 launch.
"""

import importlib.util
from pathlib import Path

import pytest
import yaml


PACKAGE = Path(__file__).resolve().parents[1]
LAUNCH = PACKAGE / 'launch'
PERCEPTION = PACKAGE.parent / 'wvcsc_perception'


def _source(name):
    return (LAUNCH / name).read_text(encoding='utf-8')


def _launch_module(name):
    path = LAUNCH / name
    spec = importlib.util.spec_from_file_location(
        f'wvcsc_bringup_dynamic_{path.stem}', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize('path', sorted(LAUNCH.glob('*.launch.py')))
def test_launch_modules_import_on_ros_humble(path):
    """Catch launch actions that are unavailable in the deployed ROS distro."""
    spec = importlib.util.spec_from_file_location(
        f'wvcsc_bringup_test_{path.stem}', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)


def test_navigation_matches_the_validated_single_command_stack():
    source = _source('real_navigation.launch.py')

    assert 'cartographer' not in source.lower()
    assert 'start_wtb_car_fdimu.launch.py' in source
    assert 'real_sensors.launch.py' not in source
    assert 'wtb_nav2_params.yaml' in source
    assert "'tf_buffer_size': '300'" in source
    assert "'start_vehicle_stack', default_value='true'" in source
    assert "'use_rviz', default_value='true'" in source
    assert "'rviz_goal_topic'" in source
    assert "default_value='/goal_pose'" in source
    assert "'open_rviz': 'false'" in source
    assert source.count("package='rviz2'") == 1
    assert 'real_navigation.rviz' in source
    assert "remappings=[('/manual_goal_pose', rviz_goal_topic)]" in source
    assert 'latest_map_yaml' in source
    assert 'orchard.yaml' not in source
    assert 'SetRemap' not in source
    assert 'bringup_launch.py' in source


def test_cartographer_launch_owns_the_complete_mapping_hardware_chain():
    source = _source('real_cartographer.launch.py')

    assert 'start_wtb_car_fdimu.launch.py' in source
    assert "executable='cartographer_node'" in source
    assert "executable='cartographer_occupancy_grid_node'" in source
    assert "remappings=[('/odom', '/ekf_odom')]" in source
    assert "default_value='cartographer.lua'" in source
    assert "default_value='0.05'" in source
    assert "default_value='0.5'" in source
    assert 'real_cartographer.rviz' in source


def test_real_system_starts_each_hardware_stack_once_after_preflight():
    source = _source('real_system_mission.launch.py')

    assert 'TimerAction' not in source
    assert "_include(launch_dir, 'real_sensors.launch.py'" in source
    assert "_include(launch_dir, 'real_navigation.launch.py'" in source
    assert "'start_vehicle_stack': 'false'" in source
    assert "'rviz_goal_topic': '/manual_goal_pose'" in source
    assert "'real_arm.launch.py', {" in source
    assert "'use_rviz': LaunchConfiguration('use_moveit_rviz')" in source
    assert "_include(launch_dir, 'real_orchestration.launch.py'" in source
    assert "'nozzle_calibration'" in source
    assert "'--require-nozzle-calibration', 'true'" in source
    assert "'nozzle_mount_xyz'" in source
    assert "'aim_trim_u_px'" in source
    assert "'relay_config_file'" in source
    assert "'--relay-config', relay_config" in source
    assert "'--operation', 'qt_mission'" in source
    assert 'mission_mode' not in source
    assert 'mission_file' not in source
    assert "'use_qt_gui', default_value='true'" in source
    assert 'nav2_qt.launch.py' in source
    assert "'use_nav_rviz', default_value='true'" in source
    assert "'use_moveit_rviz', default_value='false'" in source


def test_real_system_rviz_processes_have_separate_explicit_controls_and_names():
    system = _source('real_system_mission.launch.py')
    navigation = _source('real_navigation.launch.py')
    arm = _source('real_arm.launch.py')

    assert "'use_rviz': LaunchConfiguration('use_nav_rviz')" in system
    assert "name='real_navigation_rviz'" in navigation
    assert "name='moveit_rviz'" in arm


def test_real_orchestration_uses_the_qt_mission_manager_only():
    source = _source('real_orchestration.launch.py')
    vision_path = (PERCEPTION / 'wvcsc_rgb_vision' / 'config' /
                   'vision_real.yaml')
    vision = vision_path.read_text(encoding='utf-8')
    vision_parameters = yaml.safe_load(vision)[
        'wvcsc_perception_pipeline']['ros__parameters']

    assert "package='wvcsc_mission_manager', executable='mission_manager'" in source
    assert "executable='field_route_manager.py'" not in source
    assert 'mission_mode' not in source
    assert 'mission_file' not in source
    assert 'mission_source' not in source
    assert 'wvcsc_uav_gateway' not in source
    assert "'wide_relay_channel': 1" in source
    assert "'arm_relay_channel': 2" in source
    assert "'arm_base_yaw_rad': 3.141592653589793" in source
    assert 'vision_real.yaml' in source
    assert 'yolov8s_seg_real.pt' in vision
    assert vision_parameters['disease_model_backend'] == 'segment'
    assert 'target_class_name: diseased_target' in vision
    assert 'target_id_prefix: target' in vision
    assert 'strict_model_classes: true' in vision
    assert vision_parameters['max_diseased_targets'] == 2
    assert "get_package_share_directory('controller_pkg')" in source
    assert 'spray_actuator_real.yaml' in source
    assert "'relay_config_file'" in source


def test_qt_real_mode_uses_qt_task_autostart_without_manager_auto_start():
    system = _source('real_system_mission.launch.py')
    orchestration = _source('real_orchestration.launch.py')
    preflight = (PACKAGE / 'scripts' / 'preflight_check.py').read_text(
        encoding='utf-8')

    assert "'auto_start'" not in orchestration
    assert 'latest_field_route' not in system
    assert 'mission_file' not in system
    assert 'mission_mode' not in system
    assert "choices=('survey', 'qt_mission')" in preflight


def test_vehicle_relay_qt_test_uses_real_navigation_and_fake_arm_only():
    source = _source('real_vehicle_relay_qt_test.launch.py')
    fake = (PACKAGE / 'scripts' / 'fake_arm_spray_action.py').read_text(
        encoding='utf-8')

    assert 'real_navigation.launch.py' in source
    assert "controller.launch.py" in source
    assert "executable='fake_arm_spray_action.py'" in source
    assert "package='wvcsc_mission_manager', executable='mission_manager'" in source
    assert 'nav2_qt.launch.py' in source
    assert "'auto_start'" not in source
    assert "'arm_base_yaw_rad': math.pi" in source
    assert 'mission_file' not in source
    for forbidden in (
            'real_arm.launch.py', 'spray_task', 'perception_pipeline',
            'c10_camera.launch.py', 'visual_servo'):
        assert forbidden not in source
    assert "'/arm/execute_spray'" in fake
    assert 'relay channel 2' in fake
    assert 'self._relay_channel = 2' not in fake


def test_real_bringup_removes_file_route_and_cli_tools():
    cmake = (PACKAGE / 'CMakeLists.txt').read_text(encoding='utf-8')
    for name in (
            'capture_site_pose.py', 'migrate_site_mission.py',
            'validate_site_mission.py', 'validate_field_route.py',
            'field_route_manager.py', 'arm_spray_once.py'):
        assert name not in cmake
        assert not (PACKAGE / 'scripts' / name).exists()
    assert not (PACKAGE / 'wvcsc_bringup' / 'site_mission.py').exists()
    assert not (PACKAGE / 'wvcsc_bringup' / 'field_route.py').exists()


def test_real_arm_spray_test_is_decoupled_from_vehicle_navigation():
    source = _source('real_arm_spray_test.launch.py')
    script = (PACKAGE / 'scripts' / 'arm_spray_test_qt.py').read_text(
        encoding='utf-8')

    assert 'real_navigation.launch.py' not in source
    assert 'real_sensors.launch.py' not in source
    assert 'mission_manager' not in source
    assert 'wtb_car_driver' not in source
    assert 'lslidar_driver' not in source
    assert 'yesense_std_ros2' not in source
    assert "'publish_robot_state': 'true'" in source
    assert "_load_calibrated_mount(handeye_path)" in source
    assert "'aim_nozzle_frame': 'tool0'" in source
    assert 'nozzle_calibration' not in source
    assert 'latest_real' in source
    assert 'nozzle_xyz = (0.0, 0.0, 0.0)' in source
    assert 'c10_camera.launch.py' in source
    assert 'vision_real.yaml' in source
    assert "executable='spray_task'" in source
    assert "executable='spray_actuator'" in source
    assert 'spray_actuator_real.yaml' in source
    assert "get_package_share_directory('controller_pkg')" in source
    assert "'relay_config_file'" in source

    assert "executable='arm_spray_test_qt.py'" in source
    assert "'use_qt_gui', default_value='true'" in source
    assert 'MissionStatus.ARM_SPRAYING' in script
    assert "self.declare_parameter('base_frame', 'alicia_base_link')" in script
    assert "String(data='stop')" in script
    assert "String(data='reset')" in script
    assert "String(data='resume')" in script
    assert 'ActionClient(self, ExecuteSpray, \'/arm/execute_spray\')' in script


def test_real_arm_spray_server_wrapper_has_explicit_action_mode():
    wrapper = PACKAGE.parent / 'run_real_arm_spray_server.sh'
    source = wrapper.read_text(encoding='utf-8')

    assert wrapper.stat().st_mode & 0o111
    assert 'real_arm_spray_test.launch.py' in source
    assert 'auto_execute=false' in source
    assert 'auto_execute:=true' in source
    assert 'ros2 action send_goal /arm/execute_spray' in source
    assert 'wvcsc_interfaces/action/ExecuteSpray' in source
    assert '"$goal" -f' in source
    assert "use_qt_gui:=false \"${launch_args[@]}\"" in source
    assert 'observation_mode_launch_arg' in source
    assert 'launch_args+=("$observation_mode_launch_arg")' in source
    for argument in (
            'observation_mode', 'auto_side', 'auto_tree_distance_m',
            'auto_working_range_m', 'auto_spray_duration_sec',
            'auto_mission_id', 'auto_tree_id'):
        assert f'{argument}:=' in source

    # The wrapper may document downstream topics, but it must not implement
    # their behavior itself; SprayTask remains the single execution owner.
    assert "'/servo_node/delta_twist_cmds'" not in source
    assert "'/relay/set'" not in source
    assert "'/motion_control/command'" not in source


def test_real_arm_spray_server_goal_validation_and_side_mapping_are_explicit():
    source = (PACKAGE.parent / 'run_real_arm_spray_server.sh').read_text(
        encoding='utf-8')
    assert 'auto_side must be left or right' in source
    assert 'auto_tree_distance_m must be within 0.80-1.50 m' in source
    assert 'auto_working_range_m must be 0 or within 0.20-2.00 m' in source
    assert 'auto_spray_duration_sec must be within 0.20-10.00 s' in source
    assert 'frame_id: alicia_base_link' in source
    assert 'y: ${tree_y}' in source
    assert 'Action Server' in source


def test_spray_workflow_doc_is_first_person_and_separates_spray_triggers():
    document = (PACKAGE.parent / 'docs' / '喷洒任务全流程.md').read_text(
        encoding='utf-8')
    assert '## 4. 我如何区分两种喷洒触发' in document
    assert 'wide_spray_on_approach' in document
    assert 'wide_spray_motion_linear_threshold' in document
    assert '0.03 m/s' in document
    assert '通道 1' in document
    assert '`point_type=INSPECT`' in document
    assert '`/arm/execute_spray` Action' in document
    assert '我不会因为病态目标检测结果而触发广域喷洒' in document


def test_arm_test_exposes_separate_ik_and_manual_working_ranges():
    source = _source('real_arm_spray_test.launch.py')

    for argument, default in (
            ('working_range_min_m', '0.20'),
            ('working_range_max_m', '2.00'),
            ('default_working_range_m', '1.00'),
            ('joint_preset_hint_distance_m', '1.00')):
        assert f"'{argument}', default_value='{default}'" in source
        assert f"LaunchConfiguration('{argument}')" in source


def test_real_joint_preset_observation_mode_is_default_and_can_be_overridden():
    real_config = yaml.safe_load((PACKAGE / 'config' / 'real' /
                                  'arm_task_real.yaml').read_text(
                                      encoding='utf-8'))
    parameters = real_config['wvcsc_spray_task']['ros__parameters']
    assert parameters['observation_mode'] == 'joint_presets'
    assert parameters['spray_on_alignment_failure'] is True
    assert parameters['max_targets_per_tree'] == 2
    assert parameters['target_recenter_workspace_px'] == 128.0
    assert parameters['visual_servo_entry_max_error_px'] == 48.0
    assert parameters['target_post_recenter_stable_sec'] == 0.50
    assert parameters['target_recenter_max_angle_deg'] == 45.0
    assert parameters['target_recenter_max_total_angle_deg'] == 45.0
    assert parameters['joint_preset_center_deg'] == [
        95.3, -136.9, -71.0, 7.7, 57.3, -4.4]
    assert parameters['joint_preset_fan_left_deg'] == [
        52.2, -131.7, -55.4, -58.9, 76.5, 18.2]
    assert parameters['joint_preset_fan_right_deg'] == [
        118.5, -129.4, -55.8, 47.6, 66.2, -17.1]

    for launch_name in (
            'real_arm_spray_test.launch.py', 'real_orchestration.launch.py',
            'real_system_mission.launch.py'):
        source = _source(launch_name)
        assert "'observation_mode', default_value='joint_presets'" in source
        assert "LaunchConfiguration('observation_mode')" in source

    mission_source = _source('real_system_mission.launch.py')
    assert "'default_arm_spray_duration_sec', default_value='3.0'" in mission_source
    assert "'default_arm_spray_duration_sec': LaunchConfiguration(" in mission_source
    for launch_name in ('real_arm_spray_test.launch.py',
                        'real_orchestration.launch.py'):
        source = _source(launch_name)
        assert "os.path.join(real_config, 'arm_task_real.yaml')" in source

    simulation_config = (PACKAGE.parent / 'wvcsc_manipulation' / 'wvcsc_arm_task' / 'config' /
                         'arm_task.yaml').read_text(encoding='utf-8')
    assert 'joint_preset_center_deg' not in simulation_config
    assert 'spray_on_alignment_failure' not in simulation_config
    assert 'max_targets_per_tree: 0' in simulation_config
    config_source = (PACKAGE.parent / 'wvcsc_manipulation' / 'wvcsc_arm_task' / 'wvcsc_arm_task' /
                     'spray_config.py').read_text(encoding='utf-8')
    assert "'spray_on_alignment_failure': False" in config_source
    task_source = (PACKAGE.parent / 'wvcsc_manipulation' / 'wvcsc_arm_task' / 'wvcsc_arm_task' /
                   'spray_task.py').read_text(encoding='utf-8')
    assert 'SINGLE_SHOT_OPEN_LOOP_ALIGN' not in task_source


def test_real_spray_actuator_uses_the_physical_relay_service():
    """The real arm entry point must never silently use the timer simulator."""
    config = yaml.safe_load((PACKAGE / 'config' / 'real' /
                             'spray_actuator_real.yaml').read_text(
                                 encoding='utf-8'))
    parameters = config['wvcsc_spray_actuator']['ros__parameters']
    assert parameters['spray_mode'] == 'service'
    assert parameters['relay_service_name'] == '/relay/set'
    assert parameters['relay_channel'] == 2
    assert parameters['relay_service_timeout_sec'] > 0.0

    source = _source('real_arm_spray_test.launch.py')
    assert "executable='spray_actuator'" in source
    assert 'spray_actuator_real.yaml' in source
    assert "config_file': LaunchConfiguration('relay_config_file')" in source


def test_real_sensor_stack_has_one_unified_robot_state_publisher():
    source = _source('real_sensors.launch.py')

    assert source.count("package='robot_state_publisher'") == 1
    assert 'start_wtb_car_fdimu.launch.py' not in source
    assert "package='can_bridge'" in source
    assert "package='wtb_car_driver'" in source
    assert "package='wvcsc_safety'" not in source
    assert "executable='safety_gate'" not in source
    assert "('/twist_cmd', '/wvcsc_bringup/disabled_twist_cmd')" in source


def test_real_lidar_publishes_one_direct_scan_for_amcl_and_nav2():
    real_sensors = _source('real_sensors.launch.py')
    vehicle_launch = (
        PACKAGE.parent / 'wvcsc_vehicle' / 'wtb_car_driver' / 'launch' /
        'start_wtb_car_fdimu.launch.py').read_text(encoding='utf-8')
    lidar_driver = (
        PACKAGE.parent / 'wvcsc_vehicle' / 'lidar_ros2' / 'lslidar_ros' / 'lslidar_driver' /
        'src' / 'lslidar_driver.cpp').read_text(encoding='utf-8')

    for source in (real_sensors, vehicle_launch):
        assert "('scan', '/scan')" in source
        assert 'vehicle_scan_self_filter' not in source
        assert '/scan_unfiltered' not in source
    assert 'create_publisher<sensor_msgs::msg::LaserScan>("/scan_raw", 10)' in lidar_driver


def test_calibrated_xacro_vectors_are_quoted_for_negative_components():
    for launch_name in ('real_sensors.launch.py',
                        'real_orchestration.launch.py',
                        'real_arm.launch.py'):
        source = _source(launch_name)
        for argument in ('c10_mount_xyz', 'c10_mount_rpy',
                         'nozzle_mount_xyz', 'nozzle_mount_rpy'):
            assert f' {argument}:="' in source


def test_real_sensor_stack_uses_yesense_and_keeps_fdilink_only_for_rollback():
    for name in ('real_sensors.launch.py',):
        source = _source(name)
        assert 'yesense_std_ros2' in source
        assert 'yesense_node.launch.py' in source
        assert not any(
            'fdilink_ahrs' in line and not line.lstrip().startswith('#')
            for line in source.splitlines())

    vehicle_source = (
        PACKAGE.parent / 'wvcsc_vehicle' / 'wtb_car_driver' / 'launch' /
        'start_wtb_car_fdimu.launch.py').read_text(encoding='utf-8')
    assert 'yesense_std_ros2' in vehicle_source
    assert 'yesense_node.launch.py' in vehicle_source
    assert any(
        'fdilink_ahrs' in line and line.lstrip().startswith('#')
        for line in vehicle_source.splitlines())


def test_packaged_map_directory_exists():
    assert (PACKAGE / 'maps').is_dir()


def test_bringup_rviz_configs_copy_the_field_validated_configs():
    rviz_dir = PACKAGE / 'rviz'
    assert (rviz_dir / 'real_navigation.rviz').read_bytes() == (
        PACKAGE.parent / 'wvcsc_navigation' / 'my_navigation2' / 'rviz' /
        'nav2_default_view2.rviz').read_bytes()
    assert (rviz_dir / 'real_cartographer.rviz').read_bytes() == (
        PACKAGE.parent / 'wvcsc_navigation' / 'my_cartographer' / 'rviz' /
        'my_cartographer.rviz').read_bytes()


def test_handeye_session_enables_standalone_robot_tf():
    arm = _source('real_arm.launch.py')
    handeye = (PERCEPTION / 'wvcsc_calibration' / 'launch' /
               'c10_handeye.launch.py').read_text(encoding='utf-8')

    assert "'publish_robot_state', default_value='false'" in arm
    assert "'publish_robot_state': 'true'" in handeye
    assert "executable='motion_control'" in handeye
    assert "package='easy_handeye2', executable='handeye_server'" in handeye
    assert 'rqt_calibrator' not in handeye
    assert 'real_sensors.launch.py' not in handeye
    assert 'lslidar_driver' not in handeye
    assert 'fdilink_ahrs' not in handeye
    assert 'wtb_car_driver' not in handeye
    assert 'package://wvcsc_c10_camera/config/c10_intrinsics.yaml' in handeye

    collector = (PERCEPTION / 'wvcsc_calibration' /
                 'wvcsc_calibration' /
                 'auto_calibration_collector.py').read_text(encoding='utf-8')
    assert "RemoveSample.Request(sample_index=count - 1)" in collector
    assert "'minimum_solution_samples': 14" in collector
    assert 'calibration_output_dir' in collector
    assert 'timestamped_calibration_paths' in collector


def test_arm_only_bringup_does_not_require_unpublished_wheel_states():
    arm = _source('real_arm.launch.py')
    spray = _source('real_arm_spray_test.launch.py')
    car_xacro = (PACKAGE.parent / 'wvcsc_vehicle' / 'wtb_car_driver' / 'urdf' /
                 'wtb_car.xacro').read_text(encoding='utf-8')

    assert "DeclareLaunchArgument('enable_ackermann', default_value='false')" in arm
    assert "LaunchConfiguration('enable_ackermann')" in arm
    assert 'enable_ackermann:=false' in spray
    assert '<xacro:if value="$(arg enable_ackermann)">' in car_xacro


def test_vehicle_mapping_launch_declares_and_passes_ackermann_switch():
    vehicle_launch = (PACKAGE.parent / 'wvcsc_vehicle' / 'wtb_car_driver' / 'launch' /
                      'start_wtb_car_fdimu.launch.py').read_text(encoding='utf-8')
    cartographer = _source('real_cartographer.launch.py')
    assert "LaunchConfiguration('enable_ackermann')" in vehicle_launch
    assert "'enable_ackermann'," in vehicle_launch
    assert 'enable_ackermann:=' in vehicle_launch
    assert "'enable_ackermann': 'true'" in cartographer


def test_real_mission_uses_portable_handeye_and_c10_calibration_paths():
    source = _source('real_system_mission.launch.py')
    assert 'def _expand_path(path):' in source
    assert 'os.path.expandvars' in source
    assert "def _latest_handeye_calibration" in source
    assert "default_value='latest_real'" in source
    assert "c10_share, 'config', 'c10_intrinsics.yaml'" in source
    assert 'latest_field_route' not in source
    assert 'latest_map_yaml' in source
    assert "'wvcsc_perception' / 'wvcsc_calibration' / 'config'" in source
    assert 'nozzle.example.yaml' in source
    assert '.ros/wvcsc_calibration/nozzle.yaml' not in source


def test_real_launches_use_timestamped_defaults_not_legacy_paths():
    for launch_name in (
            'real_navigation.launch.py',
            'real_vehicle_relay_qt_test.launch.py',
            'real_orchestration.launch.py',
            'real_system_mission.launch.py'):
        source = _source(launch_name)
        assert 'wvcsc_sites' not in source
        assert 'maps/orchard.yaml' not in source
    assert not (PACKAGE / 'maps' / 'orchard.yaml').exists()
    assert not (PACKAGE / 'maps' / 'orchard.pgm').exists()
    assert not (PACKAGE / 'config' / 'real' / 'field_route_corn.example.yaml').exists()


def test_real_hardware_defaults_match_field_computer():
    for launch_name in (
            'real_arm_spray_test.launch.py',
            'real_sensors.launch.py',
            'real_system_mission.launch.py'):
        source = _source(launch_name)
        assert "default_value='/dev/video2'" in source
        assert "default_value='/dev/ttyACM0'" in source
    for launch_name in ('real_arm.launch.py', 'real_orchestration.launch.py'):
        source = _source(launch_name)
        assert "default_value='/dev/ttyACM0'" in source
    source = (PERCEPTION / 'wvcsc_calibration' / 'launch' /
              'real_vision_test.launch.py').read_text(encoding='utf-8')
    assert "default_value='/dev/video2'" in source
    for launch_name in (
            'auto_handeye.launch.py', 'c10_handeye.launch.py',
            'calibrate.launch.py', 'evaluate.launch.py'):
        source = (PERCEPTION / 'wvcsc_calibration' / 'launch' /
                  launch_name).read_text(encoding='utf-8')
        assert "default_value='/dev/video2'" in source
        assert "default_value='/dev/ttyACM0'" in source
    assert "default_value='/dev/video2'" in (
        PERCEPTION / 'wvcsc_c10_camera' / 'launch' /
        'c10_camera.launch.py').read_text(encoding='utf-8')
    assert 'video_device: /dev/video2' in (
        PERCEPTION / 'wvcsc_c10_camera' / 'config' /
        'c10_usb_cam.yaml').read_text(encoding='utf-8')


def test_tool0_coincident_nozzle_example_is_valid():
    data = yaml.safe_load((PERCEPTION / 'wvcsc_calibration' / 'config' /
                           'nozzle.example.yaml').read_text(encoding='utf-8'))
    assert data['parent_frame'] == 'tool0'
    assert data['child_frame'] == 'spray_nozzle_link'
    assert data['translation'] == {'x': 0.0, 'y': 0.0, 'z': 0.0}
    assert data['rotation'] == {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}


def test_nozzle_frame_and_compensated_aim_are_wired_through_real_stack():
    xacro = (PACKAGE.parent / 'wvcsc_description' / 'urdf' /
             'wvcsc_utb_alicia.urdf.xacro').read_text(encoding='utf-8')
    orchestration = _source('real_orchestration.launch.py')
    for launch_name in (
            'real_sensors.launch.py', 'real_arm.launch.py',
            'real_orchestration.launch.py'):
        source = _source(launch_name)
        assert "LaunchConfiguration('nozzle_mount_xyz')" in source
        assert "LaunchConfiguration('nozzle_mount_rpy')" in source
    assert '<link name="spray_nozzle_link">' in xacro
    assert '<parent link="tool0"/>' in xacro
    assert '<geometry><cylinder radius="0.012" length="0.035"/></geometry>' in xacro
    assert "'desired_offset_u_px'" in orchestration


def test_nozzle_calibration_contract_is_strict(tmp_path):
    module = _launch_module('real_system_mission.launch.py')
    path = tmp_path / 'nozzle.yaml'
    path.write_text(yaml.safe_dump({
        'schema_version': 1,
        'parent_frame': 'tool0',
        'child_frame': 'spray_nozzle_link',
        'translation': {'x': 0.01, 'y': 0.0, 'z': 0.0},
        'rotation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0},
        'working_distance_m': 1.0,
        'working_distance_tolerance_m': 0.05,
        'pixel_trim': {'u': 1.0, 'v': -2.0},
    }), encoding='utf-8')
    xyz, _rpy, distance, tolerance, trim = (
        module._load_nozzle_calibration(path))
    assert xyz == pytest.approx((0.01, 0.0, 0.0))
    assert distance == pytest.approx(1.0)
    assert tolerance == pytest.approx(0.05)
    assert trim == pytest.approx((1.0, -2.0))

    data = yaml.safe_load(path.read_text(encoding='utf-8'))
    data['child_frame'] = 'camera_link'
    path.write_text(yaml.safe_dump(data), encoding='utf-8')
    with pytest.raises(RuntimeError, match='spray_nozzle_link'):
        module._load_nozzle_calibration(path)

    data['child_frame'] = 'spray_nozzle_link'
    data['translation']['x'] = 0.31
    path.write_text(yaml.safe_dump(data), encoding='utf-8')
    with pytest.raises(RuntimeError, match='0.30 m'):
        module._load_nozzle_calibration(path)


def test_handeye_loader_accepts_raw_easy_handeye_calibration(tmp_path):
    module = _launch_module('real_system_mission.launch.py')
    path = tmp_path / 'wvcsc_c10.calib'
    path.write_text(yaml.safe_dump({
        'parameters': {
            'calibration_type': 'eye_in_hand',
            'robot_base_frame': 'alicia_base_link',
            'robot_effector_frame': 'tool0',
            'tracking_base_frame': 'camera_color_optical_frame',
        },
        'transform': {
            'translation': {'x': 0.008, 'y': -0.021, 'z': -0.103},
            'rotation': {
                'x': -0.00145, 'y': -0.02123,
                'z': -0.57547, 'w': 0.81755,
            },
        },
    }), encoding='utf-8')
    xyz, rpy = module._load_calibrated_mount(path)
    assert xyz == pytest.approx((0.008, -0.021, -0.103))
    assert all(abs(value) < 3.2 for value in rpy)

def test_preflight_checks_every_direct_runtime_boundary():
    source = (PACKAGE / 'scripts' / 'preflight_check.py').read_text(
        encoding='utf-8')

    for package in (
            'robot_localization', 'pointcloud_to_laserscan',
            'robot_state_publisher', 'xacro', 'usb_cam',
            'controller_manager', 'rclcpp_components', 'alicia_m_bringup',
            'trajectory_retime_server', 'wvcsc_description'):
        assert repr(package) in source
    assert "--camera-info" in source
    assert "--handeye-calibration" in source
    assert "--nozzle-calibration" in source
    assert "_calibration_checks(args, failures)" in source


def test_arm_base_docking_offset_matches_integrated_urdf():
    workspace = PACKAGE.parent
    mission = yaml.safe_load((
        workspace / 'wvcsc_mission_manager' / 'config' /
        'mission_manager.yaml').read_text(encoding='utf-8'))
    parameters = mission['mission_manager']['ros__parameters']
    xacro = (
        workspace / 'wvcsc_description' / 'urdf' /
        'wvcsc_utb_alicia.urdf.xacro').read_text(encoding='utf-8')

    assert parameters['arm_base_forward_offset_m'] == pytest.approx(-0.40)
    assert parameters['arm_base_left_offset_m'] == pytest.approx(0.0)
    assert 'name="arm_mount_xyz" default="-0.40 0 0"' in xacro


def test_real_arm_keeps_camera_clearance_and_servo_collision_checks():
    arm_parameters = yaml.safe_load((
        PACKAGE / 'config' / 'real' / 'arm_task_real.yaml'
    ).read_text(encoding='utf-8'))['wvcsc_spray_task']['ros__parameters']
    servo_parameters = yaml.safe_load((
        PACKAGE / 'config' / 'real' / 'moveit_servo_real.yaml'
    ).read_text(encoding='utf-8'))
    visual_parameters = yaml.safe_load((
        PACKAGE / 'config' / 'real' / 'visual_servo_real.yaml'
    ).read_text(encoding='utf-8'))['wvcsc_visual_servo']['ros__parameters']

    assert arm_parameters['camera_height_min_m'] == pytest.approx(0.15)
    assert arm_parameters['camera_height_max_m'] == pytest.approx(0.30)
    assert arm_parameters['camera_height_step_m'] == pytest.approx(0.10)
    assert 'observation_min_camera_z_in_base_m' not in arm_parameters
    assert servo_parameters['use_gazebo'] is False
    assert servo_parameters['check_collisions'] is True
    assert servo_parameters['publish_joint_velocities'] is True
    assert visual_parameters['angular_u_sign'] == pytest.approx(-1.0)
    assert visual_parameters['angular_v_sign'] == pytest.approx(1.0)
    assert visual_parameters['direction_guard_enabled'] is True
    assert visual_parameters['fine_tolerance_px'] == pytest.approx(8.0)
    assert visual_parameters['control_resume_tolerance_px'] == pytest.approx(8.0)
