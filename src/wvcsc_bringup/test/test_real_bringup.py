"""Static architecture contracts for the real WVCSC launch boundary.

These tests intentionally inspect launch sources instead of starting hardware.
They catch accidental reintroduction of Cartographer into localization mode,
duplicate hardware stacks in mapping mode, and bypasses around the velocity
safety gate.
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


def test_navigation_is_localization_only_and_routes_through_safety_gate():
    source = _source('real_navigation.launch.py')

    assert 'cartographer' not in source.lower()
    assert "'slam': 'False'" in source
    assert "SetRemap(src='/cmd_vel', dst='/cmd_vel_nav')" in source
    assert "SetRemap(src='/odom', dst='/ekf_odom')" in source
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


def test_system_modes_are_mutually_exclusive_without_timer_startup():
    source = _source('system_real.launch.py')

    assert "mode not in {'localization', 'mapping'}" in source
    assert 'mode must be localization or mapping' in source
    assert "operation not in {'survey', 'mission'}" in source
    assert 'operation must be survey or mission' in source
    assert "mission_source not in {'measured', 'uav'}" in source
    assert 'TimerAction' not in source
    assert "_include(launch_dir, 'real_cartographer.launch.py')" in source
    assert "_include(launch_dir, 'real_sensors.launch.py'" in source
    assert "_include(launch_dir, 'real_navigation.launch.py'" in source
    assert "'real_arm.launch.py', shared_description_args" in source
    assert "_include(launch_dir, 'real_orchestration.launch.py'" in source
    assert "'nozzle_calibration'" in source
    assert "'require_nozzle_calibration', default_value='true'" in source
    assert "'nozzle_mount_xyz'" in source
    assert "'aim_fixed_range_m'" in source
    assert 'use_handeye_calibration cannot be disabled in mission mode' in source
    assert 'require_nozzle_calibration cannot be disabled in mission mode' in source


def test_real_orchestration_uses_real_leaf_and_measured_mission_contracts():
    source = _source('real_orchestration.launch.py')
    vision = (PACKAGE.parent / 'wvcsc_rgb_vision' / 'config' /
              'vision_real.yaml').read_text(encoding='utf-8')

    assert "executable='load_site_mission.py'" in source
    assert "LaunchConfiguration('mission_source')" in source
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
    assert "package='wvcsc_safety'" in source
    assert "executable='safety_gate'" in source
    assert "('/twist_cmd', '/safety/disabled_twist_cmd')" in source


def test_handeye_session_enables_standalone_robot_tf():
    arm = _source('real_arm.launch.py')
    handeye = (PACKAGE.parent / 'wvcsc_calibration' / 'launch' /
               'c10_handeye.launch.py').read_text(encoding='utf-8')

    assert "'publish_robot_state', default_value='false'" in arm
    assert "'publish_robot_state': 'true'" in handeye
    assert "executable='motion_control'" in handeye
    assert "package='easy_handeye2', executable='handeye_server'" in handeye
    assert 'rqt_calibrator' not in handeye

    collector = (PACKAGE.parent / 'wvcsc_calibration' /
                 'wvcsc_calibration' /
                 'auto_calibration_collector.py').read_text(encoding='utf-8')
    assert "RemoveSample.Request(sample_index=count - 1)" in collector
    assert "'minimum_solution_samples': 14" in collector


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
    module = _launch_module('system_real.launch.py')
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
