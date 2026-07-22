import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml
from launch import LaunchContext
from launch.actions import Shutdown


LAUNCH_SOURCE = (
    Path(__file__).parents[1] / 'launch' / 'system_sim.launch.py'
).read_text(encoding='utf-8')


def _launch_module():
    path = Path(__file__).parents[1] / 'launch' / 'system_sim.launch.py'
    spec = importlib.util.spec_from_file_location('wvcsc_system_sim_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_arm_starts_under_zero_gravity_then_restores_gravity():
    spray_handler = LAUNCH_SOURCE[
        LAUNCH_SOURCE.index('start_spray_task ='):
        LAUNCH_SOURCE.index('world_map =')
    ]
    post_spawn = LAUNCH_SOURCE[
        LAUNCH_SOURCE.index('post_spawn ='):
        LAUNCH_SOURCE.index('return LaunchDescription')
    ]

    assert "process_name='gripper_controller spawner'" in spray_handler
    assert 'success_actions=[restore_gravity, spray_task]' in spray_handler
    assert 'zero_gravity' in post_spawn
    assert 'unpause_without_arm' in post_spawn
    assert 'unpause_with_zero_gravity' not in post_spawn

    zero_gravity_handler = LAUNCH_SOURCE[
        LAUNCH_SOURCE.index('start_zero_gravity_physics ='):
        LAUNCH_SOURCE.index('start_gripper_controller =')
    ]
    assert 'on_exit=[unpause_with_zero_gravity]' in zero_gravity_handler


def test_joint_state_controller_waits_for_gazebo_control_plugin():
    assert (
        'TimerAction(period=1.5, actions=[joint_state_controller])'
        in LAUNCH_SOURCE
    )


def test_controller_chain_stops_after_a_failed_spawner():
    next_action = object()
    callback = _launch_module().process_exit_actions

    assert callback(
        SimpleNamespace(returncode=0), None,
        process_name='controller', success_actions=[next_action],
    ) == [next_action]
    failure_actions = callback(
        SimpleNamespace(returncode=1), None,
        process_name='controller', success_actions=[next_action],
    )
    assert len(failure_actions) == 1
    assert isinstance(failure_actions[0], Shutdown)


def test_arm_scaling_and_controller_service_timeout_are_explicit():
    assert "DeclareLaunchArgument('arm_velocity_scaling', default_value='0.40')" \
        in LAUNCH_SOURCE
    assert "DeclareLaunchArgument('arm_acceleration_scaling', default_value='0.50')" \
        in LAUNCH_SOURCE
    assert "'--service-call-timeout', '30.0'" in LAUNCH_SOURCE
    assert "'velocity_scaling': ParameterValue(" in LAUNCH_SOURCE
    assert "'acceleration_scaling': ParameterValue(" in LAUNCH_SOURCE


def test_arm_planner_selection_is_exposed_as_launch_arguments():
    assert "DeclareLaunchArgument('planning_pipeline_id', default_value='ompl')" \
        in LAUNCH_SOURCE
    assert "DeclareLaunchArgument('planner_id', default_value='RRTConnectFast')" \
        in LAUNCH_SOURCE
    assert "'planning_pipeline_id': planning_pipeline_id" in LAUNCH_SOURCE
    assert "'planner_id': planner_id" in LAUNCH_SOURCE


def test_simulation_loads_mock_targets_without_a_uav_gateway():
    assert "package='wvcsc_mission_manager', executable='mock_target_loader'" \
        in LAUNCH_SOURCE
    assert "DeclareLaunchArgument('use_mock_targets', default_value='true')" \
        in LAUNCH_SOURCE
    assert "mock_targets.yaml" in LAUNCH_SOURCE
    assert 'wvcsc_uav_gateway' not in LAUNCH_SOURCE
    assert 'use_replay_uav' not in LAUNCH_SOURCE


@pytest.mark.parametrize('name,value', [
    ('arm_velocity_scaling', '0'),
    ('arm_velocity_scaling', '1.01'),
    ('arm_acceleration_scaling', 'invalid'),
])
def test_invalid_arm_scaling_is_rejected_before_launch(
        name, value, monkeypatch):
    monkeypatch.setenv('ROS_LOG_DIR', '/tmp/wvcsc_launch_test')
    context = LaunchContext()
    context.launch_configurations.update({
        'arm_velocity_scaling': '0.40',
        'arm_acceleration_scaling': '0.50',
        name: value,
    })

    with pytest.raises(RuntimeError):
        _launch_module().validate_arm_scaling(context)


def test_valid_arm_scaling_passes_launch_validation(monkeypatch):
    monkeypatch.setenv('ROS_LOG_DIR', '/tmp/wvcsc_launch_test')
    context = LaunchContext()
    context.launch_configurations.update({
        'arm_velocity_scaling': '0.40',
        'arm_acceleration_scaling': '0.50',
    })

    assert _launch_module().validate_arm_scaling(context) == []


def test_simulation_loads_the_canonical_alicia_ompl_config():
    config_path = (
        Path(__file__).parents[2] / 'Alicia-M-ROS2' /
        'alicia_m_moveit_config' / 'config' / 'ompl_planning.yaml'
    )
    config = yaml.safe_load(config_path.read_text(encoding='utf-8'))

    assert "load_yaml('alicia_m_moveit_config', 'config/ompl_planning.yaml')" \
        in LAUNCH_SOURCE
    assert "'ompl': ompl_planning" in LAUNCH_SOURCE
    assert config['planning_plugin'] == 'ompl_interface/OMPLPlanner'
    assert config['arm']['default_planner_config'] == 'RRTConnectFast'
    assert set(config['arm']['planner_configs']) <= set(config['planner_configs'])
    assert config['arm']['projection_evaluator'] == 'joints(joint1,joint2)'
    assert config['arm']['enforce_joint_model_state_space'] is True
    assert 'AnytimePathShortening' in config['arm']['planner_configs']


def test_simulation_control_stack_uses_layered_100_30_hz_rates():
    source_root = Path(__file__).parents[2]
    controller_config = yaml.safe_load((
        source_root / 'wvcsc_description' / 'config' /
        'ros2_controllers.yaml'
    ).read_text(encoding='utf-8'))
    servo_config = yaml.safe_load((
        source_root / 'wvcsc_visual_servo' / 'config' /
        'moveit_servo.yaml'
    ).read_text(encoding='utf-8'))
    visual_config = yaml.safe_load((
        source_root / 'wvcsc_visual_servo' / 'config' /
        'visual_servo.yaml'
    ).read_text(encoding='utf-8'))['wvcsc_visual_servo']['ros__parameters']

    assert controller_config['controller_manager']['ros__parameters'][
        'update_rate'] == 100
    assert servo_config['publish_period'] == pytest.approx(0.05)
    assert servo_config['low_latency_mode'] is True
    assert servo_config['incoming_command_timeout'] == pytest.approx(0.30)
    assert visual_config['control_rate_hz'] == pytest.approx(30.0)
    assert visual_config['zero_command_count'] == 8
    assert visual_config['zero_command_count'] / visual_config[
        'control_rate_hz'] == pytest.approx(8.0 / 30.0)
