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


def test_launch_uses_full_moveit_controller_chain_without_rqt():
    assert "'pause': 'true'" in LAUNCH_SOURCE
    assert "'--service-call-timeout', '30.0'" in LAUNCH_SOURCE
    assert "config/kinematics.yaml" in LAUNCH_SOURCE
    assert "config/joint_limits.yaml" in LAUNCH_SOURCE
    assert "config/ompl_planning.yaml" in LAUNCH_SOURCE
    assert "config/moveit_controllers.yaml" in LAUNCH_SOURCE
    assert "executable='marker_tf'" in LAUNCH_SOURCE
    assert "executable='handeye_server'" in LAUNCH_SOURCE
    assert 'calibrate.launch.py' not in LAUNCH_SOURCE
    assert 'aruco_tf_broadcaster' not in LAUNCH_SOURCE
    assert "'robot_base_frame': 'alicia_base_link'" in LAUNCH_SOURCE
    assert "'/usr/share/gazebo-11/models'" in LAUNCH_SOURCE
    assert 'target_action=spawn' in LAUNCH_SOURCE
    assert "success_actions=[unpause]" in LAUNCH_SOURCE
    assert 'target_action=unpause' in LAUNCH_SOURCE
    assert "success_actions=[joint_state]" in LAUNCH_SOURCE
    assert 'link1="link1" link2="link6" reason="Never"' in LAUNCH_SOURCE


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


def test_calibration_world_has_visible_legs_and_a_horizontal_marker():
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
        pytest.approx([0.45, 0.0, 0.752, 1.5708, 0.0, 0.0])
    gravity = world.findtext('./world/physics/gravity')
    assert gravity.split() == ['0', '0', '0']


def test_calibration_xacro_puts_tf_mount_height_in_world_joint():
    xacro = (Path(__file__).parents[2] / 'wvcsc_calibration' / 'xacro' /
             'calibration_arm_camera.urdf.xacro').read_text(encoding='utf-8')
    assert '<child link="$(arg alicia_base_link)"/>' in xacro
    assert '<origin xyz="0 0 0.76" rpy="0 0 0"/>' in xacro
    assert '<horizontal_fov>1.2</horizontal_fov>' in xacro
    assert '<link name="$(arg alicia_base_link)"/>' not in xacro


def test_simulation_collector_profile_enables_truth_gate_and_table_surface():
    config = yaml.safe_load((Path(__file__).parents[2] /
                             'wvcsc_calibration/config/' /
                             'auto_handeye_alicia_sim.yaml').read_text(
                                 encoding='utf-8'))[
        'auto_calibration_collector']['ros__parameters']
    assert config['use_sim_time'] is True
    assert config['calibration_surface_enabled'] is True
    assert config['ground_truth_check_enabled'] is True
    assert config['auto_start'] is True
    assert config['marker_size_m'] == pytest.approx(0.070)
    assert config['ground_truth_max_translation_error_m'] == pytest.approx(0.003)
    assert config['ground_truth_max_rotation_error_deg'] == pytest.approx(1.0)
    assert config['maximum_marker_position_rms_m'] == pytest.approx(0.002)
    assert config['maximum_marker_rotation_rms_deg'] == pytest.approx(0.50)
    assert config['maximum_algorithm_translation_delta_m'] == pytest.approx(0.005)
    assert config['maximum_algorithm_rotation_delta_deg'] == pytest.approx(1.0)
