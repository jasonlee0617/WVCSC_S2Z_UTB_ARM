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


def test_simulation_uses_the_shared_service_relay_contract():
    assert "'config', 'spray_actuator.yaml'" in LAUNCH_SOURCE
    assert "package='wvcsc_simulation', executable='sim_relay.py'" in LAUNCH_SOURCE
    assert 'guard_sim_relay' in LAUNCH_SOURCE
    assert "'spray_on_alignment_failure': True" in LAUNCH_SOURCE
    assert "use_mission_manager, \"' == 'true' or '\"" in LAUNCH_SOURCE


def test_simulation_requires_a_live_relay_before_running_a_route():
    config = yaml.safe_load((
        Path(__file__).parents[2] / 'wvcsc_mission_manager' / 'config' /
        'mission_manager.yaml'
    ).read_text(encoding='utf-8'))['mission_manager']['ros__parameters']

    assert config['require_relay_service'] is True


def test_simulation_uses_plain_gzclient_instead_of_humble_eol_gui_wrapper():
    assert "'gzserver.launch.py'" in LAUNCH_SOURCE
    assert "cmd=['gzclient']" in LAUNCH_SOURCE
    assert "os.path.join(gazebo_share, 'launch', 'gazebo.launch.py')" not in LAUNCH_SOURCE
    assert 'libgazebo_ros_eol_gui.so' not in LAUNCH_SOURCE


def test_gazebo_spray_visual_uses_the_gazebo_owned_ros_executor():
    plugin = (Path(__file__).parents[1] / 'src' /
              'spray_visual_plugin.cpp').read_text(encoding='utf-8')

    assert 'gazebo_ros::Node::Get' in plugin
    assert 'rclcpp::spin_some(this->node_)' not in plugin
    assert 'std::atomic_bool wide_active_' in plugin
    assert 'Qt::WA_DontShowOnScreen' in plugin
    assert 'this->hide();' in plugin


def test_sim_navigation_uses_one_nav2_goal_checker_without_mission_docking_gate():
    behavior_tree_dir = Path(__file__).parents[1] / 'config' / 'behavior_trees'
    route_tree = (behavior_tree_dir / 'navigate_route.xml').read_text(
        encoding='utf-8')
    config = yaml.safe_load((
        Path(__file__).parents[1] / 'config' / 'nav2_sim.yaml'
    ).read_text(encoding='utf-8'))

    assert 'goal_checker_id="route_goal_checker"' in route_tree
    assert config['controller_server']['ros__parameters']['progress_checker'][
        'required_movement_radius'] == pytest.approx(0.05)
    assert config['controller_server']['ros__parameters'][
        'failure_tolerance'] == pytest.approx(2.0)
    parameters = config['controller_server']['ros__parameters']
    assert parameters['goal_checker_plugins'] == ['route_goal_checker']
    route_goal_checker = parameters['route_goal_checker']
    assert route_goal_checker['stateful'] is False
    assert config['controller_server']['ros__parameters'][
        'robot_base_frame'] == 'base_footprint'
    assert route_goal_checker['xy_goal_tolerance'] == pytest.approx(0.08)
    assert route_goal_checker['yaw_goal_tolerance'] == pytest.approx(0.12)
    assert config['planner_server']['ros__parameters']['GridBased'][
        'tolerance'] == pytest.approx(0.0)
    assert config['planner_server']['ros__parameters']['GridBased'][
        'minimum_turning_radius'] == pytest.approx(1.575)
    follow_path = config['controller_server']['ros__parameters']['FollowPath']
    assert follow_path['plugin'] == (
        'nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController')
    assert follow_path['desired_linear_vel'] == pytest.approx(0.25)
    assert follow_path['use_collision_detection'] is True
    assert follow_path['use_cost_regulated_linear_velocity_scaling'] is False
    assert 'RewrittenYaml' in LAUNCH_SOURCE
    assert 'require_docking_quality' not in LAUNCH_SOURCE
    assert 'accept_aborted_near_goal' not in LAUNCH_SOURCE
    assert 'navigate_inspect.xml' not in LAUNCH_SOURCE
    assert "'nav_goal_timeout_sec': 45.0" in LAUNCH_SOURCE
    # Nav2's velocity smoother publishes at 20 Hz. The simulator timeout must
    # leave transport jitter margin rather than matching that 50 ms period.
    assert "'command_timeout': 0.25" in LAUNCH_SOURCE
    assert '<RecoveryNode number_of_retries="4" name="NavigateRecovery">' in route_tree
    assert 'ClearEntireCostmap' in route_tree
    assert 'BackUp name="BackUpRecovery"' in route_tree
    assert 'IsPathValid path="{path}"' in route_tree
    assert '<GlobalUpdatedGoal/>' in route_tree
    assert '<Spin' not in route_tree


def test_simulation_uses_confirmed_real_vehicle_geometry_and_driver_semantics():
    assert "'wheel_base': 0.82" in LAUNCH_SOURCE
    assert "'cmd_angular_mode': 'yaw_rate'" in LAUNCH_SOURCE


def test_simulation_records_the_actual_vehicle_path_for_rviz():
    source = (Path(__file__).parents[1] / 'scripts' /
              'ackermann_sim.py').read_text(encoding='utf-8')
    rviz = (Path(__file__).parents[1] / 'rviz' / 'wvcsc.rviz').read_text(
        encoding='utf-8')

    assert "'executed_path_topic', '/vehicle/executed_path'" in source
    assert 'MissionStatus, \'/mission/status\'' in source
    assert "MissionPlan, '/mission/plan'" in source
    assert 'for point in message.points' in source
    assert "'/vehicle/route_cross_track_error'" in source
    assert "'/vehicle/controller_path_error'" in source
    assert 'def _append_executed_path' in source
    assert 'Name: Executed Vehicle Path' in rviz
    assert 'Value: /vehicle/executed_path' in rviz
    assert 'Color: 255; 0; 255' in rviz


def test_rviz_grid_matches_the_expanded_thirty_meter_square_map():
    rviz = (Path(__file__).parents[1] / 'rviz' / 'wvcsc.rviz').read_text(
        encoding='utf-8')

    assert 'Cell Size: 0.5' in rviz
    assert 'Plane Cell Count: 60' in rviz
    # 60 cells * 0.5 m are centred at x=10 m, so the grid is exactly
    # x=[-5,25] and y=[-15,15], matching orchard.yaml.
    assert 'Offset:\n        X: 10' in rviz


def test_simulation_keeps_segmentation_as_the_default_disease_backend():
    config = yaml.safe_load((
        Path(__file__).parents[2] / 'wvcsc_perception' /
        'wvcsc_rgb_vision' / 'config' / 'vision_sim.yaml'
    ).read_text(encoding='utf-8'))['wvcsc_perception_pipeline']['ros__parameters']

    assert config['disease_model_backend'] == 'segment'
    assert config['disease_model_path'] == 'yolov8s_seg_sim.pt'
    assert config['max_diseased_targets'] == 0
    assert config['target_reassociation_require_unique_candidate'] is False


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


def test_simulation_uses_qt_rviz_as_the_only_manual_task_source():
    assert 'mock_target_loader' not in LAUNCH_SOURCE
    assert 'use_mock_targets' not in LAUNCH_SOURCE
    assert 'mock_targets.yaml' not in LAUNCH_SOURCE
    assert 'auto_start_mission' not in LAUNCH_SOURCE
    assert "DeclareLaunchArgument('use_nav2_qt', default_value='true')" \
        in LAUNCH_SOURCE
    assert "DeclareLaunchArgument('use_rviz', default_value='true')" \
        in LAUNCH_SOURCE
    assert "DeclareLaunchArgument('observation_mode', default_value='ik')" \
        in LAUNCH_SOURCE
    assert "'default_arm_spray_duration_sec', default_value='3.0'" \
        in LAUNCH_SOURCE
    assert "'default_arm_spray_duration_sec': default_arm_spray_duration_sec" \
        in LAUNCH_SOURCE
    assert "'simulation_parking_clearance_check': 'true'" in LAUNCH_SOURCE
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


def test_simulation_control_stack_uses_executable_layered_rates():
    source_root = Path(__file__).parents[2]
    controller_config = yaml.safe_load((
        source_root / 'wvcsc_description' / 'config' /
        'ros2_controllers.yaml'
    ).read_text(encoding='utf-8'))
    servo_config = yaml.safe_load((
        source_root / 'wvcsc_manipulation' / 'wvcsc_visual_servo' / 'config' /
        'moveit_servo.yaml'
    ).read_text(encoding='utf-8'))
    visual_config = yaml.safe_load((
        source_root / 'wvcsc_manipulation' / 'wvcsc_visual_servo' / 'config' /
        'visual_servo.yaml'
    ).read_text(encoding='utf-8'))['wvcsc_visual_servo']['ros__parameters']

    assert controller_config['controller_manager']['ros__parameters'][
        'update_rate'] == 100
    assert servo_config['publish_period'] == pytest.approx(0.10)
    assert servo_config['low_latency_mode'] is False
    assert servo_config['use_gazebo'] is True
    # The simulated controller accepts position-only trajectories.  Sending
    # terminal velocity fields made Gazebo reject otherwise valid Servo output.
    assert servo_config['publish_joint_velocities'] is False
    assert servo_config['incoming_command_timeout'] == pytest.approx(0.30)
    assert servo_config['check_collisions'] is True
    assert visual_config['control_rate_hz'] == pytest.approx(30.0)
    assert visual_config['zero_command_count'] == 8
    assert visual_config['zero_command_count'] / visual_config[
        'control_rate_hz'] == pytest.approx(8.0 / 30.0)


def test_simulation_observation_keeps_camera_above_the_vehicle_roof():
    parameters = yaml.safe_load((
        Path(__file__).parents[2] / 'wvcsc_manipulation' / 'wvcsc_arm_task' / 'config' /
        'arm_task.yaml'
    ).read_text(encoding='utf-8'))['wvcsc_spray_task']['ros__parameters']

    assert parameters['camera_height_min_m'] == pytest.approx(0.15)
    assert parameters['camera_height_max_m'] == pytest.approx(0.40)
    assert parameters['camera_height_step_m'] == pytest.approx(0.10)
    assert parameters['observation_camera_reach_min_m'] == pytest.approx(0.2)
    assert parameters['observation_camera_reach_max_m'] == pytest.approx(0.4)
    assert parameters['observation_center_height_m'] == pytest.approx(1.3)
    assert parameters['observation_preferred_nozzle_plane_distance_m'] == pytest.approx(1.0)
    assert 'observation_min_camera_z_in_base_m' not in parameters
    assert parameters['target_recenter_trigger_px'] == pytest.approx(16.0)
    assert parameters['cross_view_reassociation_max_distance_px'] == pytest.approx(320.0)
    assert parameters['visual_servo_entry_max_error_px'] == pytest.approx(16.0)
    assert parameters['target_recenter_max_total_angle_deg'] == pytest.approx(45.0)
    assert parameters['target_recenter_max_iterations'] == 8
    assert parameters['max_alignment_attempts'] == 2
    assert parameters['target_post_recenter_stable_sec'] == pytest.approx(0.50)
    assert parameters['target_post_recenter_min_confidence'] == pytest.approx(0.30)
