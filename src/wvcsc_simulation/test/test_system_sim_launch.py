import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest
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
