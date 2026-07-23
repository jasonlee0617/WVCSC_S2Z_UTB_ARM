"""Static contracts for the standalone Alicia-M Gazebo calibration launch."""

import importlib.util
from pathlib import Path
from types import SimpleNamespace
import xml.etree.ElementTree as ElementTree

import pytest
import yaml
from launch.actions import Shutdown


ROOT = Path(__file__).parents[1]
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
    assert 'calibrate.launch.py' not in LAUNCH_SOURCE
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
    assert '# LEGACY DESK CALIBRATION ENVIRONMENT - REFERENCE ONLY' in LAUNCH_SOURCE
    assert '# legacy_xacro_file = os.path.join(' in LAUNCH_SOURCE
    assert '# legacy_world = os.path.join(' in LAUNCH_SOURCE


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


def test_legacy_calibration_world_has_visible_legs_and_a_horizontal_marker():
    world = ElementTree.parse(ROOT / 'worlds' / 'calibration_table.world').getroot()
    model = world.find("./world/model[@name='calibration_desk']")
    assert model is not None
    top_size = model.findtext("./link[@name='desk_top']/visual/geometry/box/size")
    assert top_size.split() == ['1.2', '0.8', '0.02']
    for name in ('leg_fl', 'leg_fr', 'leg_bl', 'leg_br'):
        link = model.find(f"./link[@name='{name}']")
        assert link.find('collision') is not None
        assert link.find('visual') is not None
    marker = next(item for item in world.findall('./world/include')
                  if item.findtext('name') == 'aruco_marker')
    assert [float(value) for value in marker.findtext('pose').split()] == \
        pytest.approx([0.0, 0.25, 0.752, 1.5708, 0.0, 0.0])
    gravity = world.findtext('./world/physics/gravity')
    assert gravity.split() == ['0', '0', '0']


def test_legacy_calibration_xacro_is_retained_as_a_reference_asset():
    xacro = (Path(__file__).parents[2] / 'wvcsc_calibration' / 'xacro' /
             'calibration_arm_camera.urdf.xacro').read_text(encoding='utf-8')
    assert '<child link="$(arg alicia_base_link)"/>' in xacro
    assert '<origin xyz="0 0 0.75" rpy="0 0 0"/>' in xacro
    assert 'LEGACY REFERENCE ONLY' in xacro
    assert "xacro.load_yaml('$(find wvcsc_c10_camera)/config/c10_intrinsics.yaml')" in xacro
    assert '<link name="$(arg alicia_base_link)"/>' not in xacro


def test_simulation_collector_profile_enables_truth_gate_and_vehicle_anchor():
    config = yaml.safe_load((Path(__file__).parents[2] /
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
    assert config['marker_position_base_m'] == pytest.approx([0.0, 0.25, 0.002])
    assert config['seed_height_candidates_m'] == pytest.approx(
        [0.14, 0.15, 0.16, 0.17, 0.18, 0.19, 0.20])
    assert config['seed_radial_backoff_candidates_m'] == pytest.approx(
        [0.0, 0.05, 0.10, 0.15, 0.20])
    assert config['marker_distance_min_m'] == pytest.approx(0.20)
    assert 'minimum_corner_margin_px' not in config
    assert 'use_marker_position_prior_for_candidate_generation' not in config
    assert 'maximum_center_error_px' not in config
    assert config['output_file'].startswith('$HOME/WVCSC_S2Z_UTB_ARM/src/')
    assert config['marker_size_m'] == pytest.approx(0.070)
    assert config['minimum_samples'] == 18
    assert config['minimum_solution_samples'] == 18
    assert config['minimum_safe_candidates'] == 30
    assert config['ground_truth_max_translation_error_m'] == pytest.approx(0.003)
    assert config['ground_truth_max_xy_error_m'] == pytest.approx(0.002)
    assert config['ground_truth_max_rotation_error_deg'] == pytest.approx(1.0)
    assert config['maximum_marker_position_rms_m'] == pytest.approx(0.002)
    assert config['maximum_marker_rotation_rms_deg'] == pytest.approx(0.50)
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
