"""Static contracts for the standalone Alicia-M Gazebo calibration launch."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace
import xml.etree.ElementTree as ElementTree

import pytest
import yaml
from launch.actions import Shutdown
from tf_transformations import inverse_matrix, rotation_matrix
from wvcsc_calibration.alicia_sample_geometry import (
    ALICIA_M_FIXED_JOINT_SAMPLES,
)


ROOT = Path(__file__).parents[1]
PERCEPTION_ROOT = ROOT.parent / 'wvcsc_perception'
LAUNCH_PATH = ROOT / 'launch' / 'calibration_sim.launch.py'
LAUNCH_SOURCE = LAUNCH_PATH.read_text(encoding='utf-8')


def _launch_module():
    spec = importlib.util.spec_from_file_location(
        'wvcsc_calibration_sim_launch', LAUNCH_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_robot_description_sanitizer_removes_only_xml_comments():
    source = '<robot name="a"><!-- yaml: value --><link name="base"/></robot>'
    sanitized = _launch_module()._sanitize_robot_description(source)
    assert '<!--' not in sanitized
    assert '<link name="base"/>' in sanitized


def test_launch_uses_vehicle_model_and_full_controller_chain_without_rqt():
    assert "'pause': 'true'" in LAUNCH_SOURCE
    assert "'--service-call-timeout', '30.0'" in LAUNCH_SOURCE
    assert "config/kinematics.yaml" in LAUNCH_SOURCE
    assert "config/joint_limits.yaml" in LAUNCH_SOURCE
    assert "config/ompl_planning.yaml" in LAUNCH_SOURCE
    assert "config/moveit_controllers.yaml" in LAUNCH_SOURCE
    assert "executable='marker_tf'" in LAUNCH_SOURCE
    assert "executable='visualize_aruco_marker'" in LAUNCH_SOURCE
    assert "'/calibration/aruco_debug_image'" in LAUNCH_SOURCE
    assert "executable='handeye_server'" in LAUNCH_SOURCE
    assert "executable='auto_calibration_collector'" in LAUNCH_SOURCE
    assert 'aruco_tf_broadcaster' not in LAUNCH_SOURCE
    assert "'robot_base_frame': 'alicia_base_link'" in LAUNCH_SOURCE
    assert "'/usr/share/gazebo-11/models'" in LAUNCH_SOURCE
    assert "'wvcsc_utb_alicia.urdf.xacro'" in LAUNCH_SOURCE
    assert "'calibration_vehicle.world'" in LAUNCH_SOURCE
    assert "'enable_ackermann:=true'" in LAUNCH_SOURCE
    assert "'use_collision_meshes:=true'" in LAUNCH_SOURCE
    assert "'calibration_fix_base:=true'" in LAUNCH_SOURCE
    assert "'c10_noise_stddev:=0.0'" in LAUNCH_SOURCE
    assert "'wvcsc_calibration_vehicle::base_footprint'" in LAUNCH_SOURCE
    assert "'name=\"wvcsc_utb_alicia\"'" in LAUNCH_SOURCE
    assert 'target_action=spawn_vehicle' in LAUNCH_SOURCE
    assert "success_actions=[spawn_marker]" in LAUNCH_SOURCE
    assert 'target_action=spawn_marker' in LAUNCH_SOURCE
    assert "success_actions=[unpause]" in LAUNCH_SOURCE
    assert 'target_action=unpause' in LAUNCH_SOURCE
    assert "success_actions=[joint_state]" in LAUNCH_SOURCE
    assert 'link1="link1" link2="link6" reason="Never"' not in LAUNCH_SOURCE
    assert 'calibration_table.world' not in LAUNCH_SOURCE
    assert 'calibration_arm_camera.urdf.xacro' not in LAUNCH_SOURCE


def test_marker_pose_is_transformed_from_alicia_base_to_gazebo_root():
    robot = '''
<robot name="wvcsc_utb_alicia">
  <link name="base_footprint"/><link name="base_link"/>
  <link name="arm_mount_link"/><link name="alicia_base_link"/>
  <joint name="base" type="fixed"><parent link="base_footprint"/>
    <child link="base_link"/><origin xyz="0 0 0.875" rpy="0 0 0"/></joint>
  <joint name="roof" type="fixed"><parent link="base_link"/>
    <child link="arm_mount_link"/><origin xyz="0 0 0.675" rpy="0 0 0"/></joint>
  <joint name="mount" type="fixed"><parent link="arm_mount_link"/>
    <child link="alicia_base_link"/>
    <origin xyz="-0.4 0 0" rpy="0 0 3.141592653589793"/></joint>
</robot>'''
    xyz, rpy = _launch_module()._marker_spawn_pose(robot, (0.0, 0.25, 0.002))
    assert xyz == pytest.approx((-0.4, -0.25, 1.552))
    assert rpy == pytest.approx((1.57079632679, 0.0, 3.141592653589793))


def test_coverage_marker_keeps_at_least_fourteen_fixed_poses_in_strict_c10_view():
    """Protect the table location chosen for the complete fixed sequence."""
    module = _launch_module()
    description_root = ROOT.parent / 'wvcsc_description'
    robot = module._generate_robot_description(
        description_root / 'urdf' / 'wvcsc_utb_alicia.urdf.xacro',
        description_root / 'config' / 'ros2_controllers.yaml')
    root = ElementTree.fromstring(robot)
    by_child = {
        joint.find('child').attrib['link']: joint
        for joint in root.findall('joint')
        if joint.find('child') is not None
    }
    chain = []
    link = 'camera_color_optical_frame'
    while link != 'alicia_base_link':
        joint = by_child[link]
        chain.append(joint)
        link = joint.find('parent').attrib['link']

    config = yaml.safe_load((PERCEPTION_ROOT /
                             'wvcsc_calibration/config/' /
                             'auto_handeye_alicia_sim.yaml').read_text(
                                 encoding='utf-8'))[
        'auto_calibration_collector']['ros__parameters']
    intrinsics = yaml.safe_load((PERCEPTION_ROOT / 'wvcsc_c10_camera' /
                                 'config' / 'c10_intrinsics.yaml').read_text(
                                     encoding='utf-8'))
    fx, fy, cx, cy = (
        intrinsics['camera_matrix']['data'][0],
        intrinsics['camera_matrix']['data'][4],
        intrinsics['camera_matrix']['data'][2],
        intrinsics['camera_matrix']['data'][5])
    marker = config['marker_position_base_m']
    corners = tuple((marker[0] + sign_x * 0.035,
                     marker[1] + sign_y * 0.035,
                     marker[2], 1.0)
                    for sign_x in (-1.0, 1.0) for sign_y in (-1.0, 1.0))

    valid = 0
    for sample in ALICIA_M_FIXED_JOINT_SAMPLES:
        joints = dict(zip(
            ('joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'), sample))
        transform = module.identity_matrix()
        for joint in reversed(chain):
            origin = joint.find('origin')
            xyz = tuple(float(value) for value in
                        origin.attrib.get('xyz', '0 0 0').split())
            rpy = tuple(float(value) for value in
                        origin.attrib.get('rpy', '0 0 0').split())
            transform = module.concatenate_matrices(
                transform, module.translation_matrix(xyz),
                module.euler_matrix(*rpy))
            name = joint.attrib['name']
            if name in joints:
                axis = tuple(float(value) for value in
                             joint.find('axis').attrib.get('xyz', '1 0 0').split())
                transform = module.concatenate_matrices(
                    transform, rotation_matrix(joints[name], axis))

        projected = tuple(inverse_matrix(transform).dot(corner) for corner in corners)
        if any(point[2] <= 0.02 for point in projected):
            continue
        pixels = tuple((fx * point[0] / point[2] + cx,
                        fy * point[1] / point[2] + cy) for point in projected)
        u_values, v_values = zip(*pixels)
        margin = min(min(u_values), intrinsics['image_width'] - max(u_values),
                     min(v_values), intrinsics['image_height'] - max(v_values))
        side_px = max(u_values) - min(u_values)
        center = tuple(sum(point[index] for point in projected) / len(projected)
                       for index in range(3))
        range_m = sum(value * value for value in center) ** 0.5
        if margin >= 60.0 and side_px >= 90.0 and 0.20 <= range_m <= 0.80:
            valid += 1

    assert marker == pytest.approx([0.530, -0.030, 0.002])
    assert valid >= config['minimum_samples']


def test_failed_controller_spawner_stops_the_calibration_launch():
    module = _launch_module()
    action = object()
    assert module._after_success(
        SimpleNamespace(returncode=0), None,
        process_name='controller', success_actions=[action]) == [action]
    failed = module._after_success(
        SimpleNamespace(returncode=1), None,
        process_name='controller', success_actions=[action])
    assert len(failed) == 1
    assert isinstance(failed[0], Shutdown)


def test_simulation_collector_profile_enables_truth_gate_and_vehicle_anchor():
    config = yaml.safe_load((PERCEPTION_ROOT /
                             'wvcsc_calibration/config/' /
                             'auto_handeye_alicia_sim.yaml').read_text(
                                 encoding='utf-8'))[
        'auto_calibration_collector']['ros__parameters']
    assert config['use_sim_time'] is True
    assert config['calibration_surface_enabled'] is False
    assert config['ground_truth_check_enabled'] is True
    assert config['auto_start'] is True
    assert config['velocity_scaling'] == pytest.approx(0.20)
    assert config['acceleration_scaling'] == pytest.approx(0.20)
    assert config['joint_stationary_max_position_delta_rad'] == pytest.approx(0.0001)
    assert config['joint_stationary_window_sec'] == pytest.approx(0.30)
    assert config['joint_stationary_timeout_sec'] == pytest.approx(5.0)
    # 固定20姿态覆盖率最优的 640x480 桌面位置；2 mm 是标定板表面，
    # 而非 Gazebo 模型中心。
    assert config['marker_position_base_m'] == pytest.approx([0.530, -0.030, 0.002])
    assert config['marker_distance_min_m'] == pytest.approx(0.20)
    assert 'minimum_corner_margin_px' not in config
    assert 'use_marker_position_prior_for_candidate_generation' not in config
    assert 'maximum_center_error_px' not in config
    assert config['calibration_output_dir'].startswith(
        '$HOME/WVCSC_S2Z_UTB_ARM/src/')
    assert config['calibration_file_prefix'] == 'c10_handeye_sim'
    assert config['calibration_simulation'] is True
    assert config['marker_size_m'] == pytest.approx(0.070)
    assert config['minimum_samples'] == 14
    assert config['minimum_solution_samples'] == 14
    assert 'seed_height_candidates_m' not in config
    assert 'target_samples' not in config
    assert 'maximum_samples' not in config
    assert 'minimum_safe_candidates' not in config
    assert config['ground_truth_max_translation_error_m'] == pytest.approx(0.003)
    assert config['ground_truth_max_xy_error_m'] == pytest.approx(0.002)
    assert config['ground_truth_max_rotation_error_deg'] == pytest.approx(1.0)
    assert config['maximum_marker_position_rms_m'] == pytest.approx(0.002)
    assert config['maximum_marker_rotation_rms_deg'] == pytest.approx(0.60)
    assert config['maximum_algorithm_translation_delta_m'] == pytest.approx(0.003)
    assert config['maximum_algorithm_rotation_delta_deg'] == pytest.approx(1.0)
    assert config['fixed_marker_refinement_enabled'] is True
    assert config['fixed_marker_refinement_translation_sigma_m'] == pytest.approx(0.00050)
    assert config['fixed_marker_refinement_rotation_sigma_deg'] == pytest.approx(0.30)


def test_calibration_controller_requires_zero_velocity_before_goal_completion():
    controller_config = (Path(__file__).parents[2] / 'wvcsc_description' / 'config' /
                         'ros2_controllers.yaml').read_text(encoding='utf-8')
    assert 'allow_nonzero_velocity_at_trajectory_end: false' in controller_config


def test_vehicle_calibration_world_is_minimal_and_zero_gravity():
    world = ElementTree.parse(ROOT / 'worlds' / 'calibration_vehicle.world').getroot()
    assert world.find("./world/model[@name='calibration_desk']") is None
    assert world.findtext('./world/physics/gravity').split() == ['0', '0', '0']


def test_aruco_marker_cells_match_the_declared_70mm_square():
    marker = ElementTree.parse(
        ROOT / 'models' / 'aruco_marker' / 'model.sdf').getroot()
    cells = [visual for visual in marker.findall('.//visual')
             if 'black' in visual.attrib['name']]
    assert cells
    bounds = []
    for cell in cells:
        x, _y, z, *_rpy = (float(value) for value in cell.findtext('pose').split())
        sx, _sy, sz = (float(value) for value in
                       cell.findtext('./geometry/box/size').split())
        assert sx == pytest.approx(0.010)
        assert sz == pytest.approx(0.010)
        bounds.append((x - sx / 2.0, x + sx / 2.0,
                       z - sz / 2.0, z + sz / 2.0))
    assert max(item[1] for item in bounds) - min(item[0] for item in bounds) == \
        pytest.approx(0.070)
    assert max(item[3] for item in bounds) - min(item[2] for item in bounds) == \
        pytest.approx(0.070)


def test_aruco_marker_cells_are_flush_with_the_board_render_surface():
    marker = ElementTree.parse(
        ROOT / 'models' / 'aruco_marker' / 'model.sdf').getroot()
    backing_depth = float(marker.findtext(
        ".//visual[@name='white_backing']/geometry/box/size").split()[1])
    cells = [visual for visual in marker.findall('.//visual')
             if 'black' in visual.attrib['name']]
    assert cells
    for cell in cells:
        _x, y, _z, *_rpy = (float(value) for value in cell.findtext('pose').split())
        _sx, depth, _sz = (float(value) for value in
                           cell.findtext('./geometry/box/size').split())
        protrusion = abs(y) + depth / 2.0 - backing_depth / 2.0
        assert depth == pytest.approx(0.00002)
        assert protrusion == pytest.approx(0.00002)
