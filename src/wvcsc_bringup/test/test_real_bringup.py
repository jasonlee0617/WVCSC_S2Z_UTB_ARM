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
    assert "'open_rviz': 'false'" in source
    assert source.count("package='rviz2'") == 1
    assert 'real_navigation.rviz' in source
    assert 'wvcsc_bringup/maps/orchard.yaml' in source
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
    assert "'real_arm.launch.py', shared_description_args" in source
    assert "_include(launch_dir, 'real_orchestration.launch.py'" in source
    assert "'nozzle_calibration'" in source
    assert "'--require-nozzle-calibration', 'true'" in source
    assert "'nozzle_mount_xyz'" in source
    assert "'aim_fixed_range_m'" in source


def test_real_orchestration_uses_real_leaf_and_measured_mission_contracts():
    source = _source('real_orchestration.launch.py')
    vision = (PACKAGE.parent / 'wvcsc_rgb_vision' / 'config' /
              'vision_real.yaml').read_text(encoding='utf-8')

    assert "executable='load_site_mission.py'" in source
    assert 'mission_source' not in source
    assert 'wvcsc_uav_gateway' not in source
    assert "'require_docking_quality': True" in source
    assert 'vision_real.yaml' in source
    assert 'yolov8s_real.pt' in vision
    assert 'yolov8s_seg_real.pt' in vision
    assert 'target_class_name: disease_leaf' in vision
    assert 'target_id_prefix: leaf' in vision
    assert 'strict_model_classes: true' in vision


def test_real_sensor_stack_has_one_unified_robot_state_publisher():
    source = _source('real_sensors.launch.py')

    assert source.count("package='robot_state_publisher'") == 1
    assert 'start_wtb_car_fdimu.launch.py' not in source
    assert "package='can_bridge'" in source
    assert "package='wtb_car_driver'" in source
    assert "package='wvcsc_safety'" not in source
    assert "executable='safety_gate'" not in source
    assert "('/twist_cmd', '/wvcsc_bringup/disabled_twist_cmd')" in source


def test_real_sensor_stack_uses_yesense_and_keeps_fdilink_only_for_rollback():
    for name in ('real_sensors.launch.py',):
        source = _source(name)
        assert 'yesense_std_ros2' in source
        assert 'yesense_node.launch.py' in source
        assert not any(
            'fdilink_ahrs' in line and not line.lstrip().startswith('#')
            for line in source.splitlines())

    vehicle_source = (
        PACKAGE.parent / 'wtb_car_driver' / 'launch' /
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
        PACKAGE.parent / 'my_navigation2' / 'rviz' /
        'nav2_default_view2.rviz').read_bytes()
    assert (rviz_dir / 'real_cartographer.rviz').read_bytes() == (
        PACKAGE.parent / 'my_cartographer' / 'rviz' /
        'my_cartographer.rviz').read_bytes()


def test_handeye_session_enables_standalone_robot_tf():
    arm = _source('real_arm.launch.py')
    handeye = (PACKAGE.parent / 'wvcsc_calibration' / 'launch' /
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

    collector = (PACKAGE.parent / 'wvcsc_calibration' /
                 'wvcsc_calibration' /
                 'auto_calibration_collector.py').read_text(encoding='utf-8')
    assert "RemoveSample.Request(sample_index=count - 1)" in collector
    assert "'minimum_solution_samples': 14" in collector
    assert '$HOME/WVCSC_S2Z_UTB_ARM/src/wvcsc_calibration/config/' in collector


def test_real_mission_uses_portable_handeye_and_c10_calibration_paths():
    source = _source('real_system_mission.launch.py')
    assert 'def _expand_path(path):' in source
    assert 'os.path.expandvars' in source
    assert '$HOME/WVCSC_S2Z_UTB_ARM/src/wvcsc_calibration/config/' in source
    assert "c10_share, 'config', 'c10_intrinsics.yaml'" in source


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
    assert '<link name="spray_nozzle_link"/>' in xacro
    assert '<parent link="tool0"/>' in xacro
    assert "'aim_range_tolerance_m'" in orchestration
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
