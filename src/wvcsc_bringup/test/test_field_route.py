from pathlib import Path

import pytest
import yaml

from wvcsc_bringup.field_route import (
    ALICIA_ARM_BASE_YAW_RAD,
    ARM_SPRAY_DURATION_SEC,
    ROUTE_POINT_IDS,
    ROUTE_ROLES,
    load_field_route_document,
    new_field_route_document,
    route_steps,
    validate_field_route_document,
)


PACKAGE = Path(__file__).resolve().parents[1]


def _map(tmp_path):
    image = tmp_path / 'map.pgm'
    image.write_bytes(b'P5\n100 100\n255\n' + bytes([254]) * 10000)
    yaml_path = tmp_path / 'map.yaml'
    yaml_path.write_text(
        'image: map.pgm\nresolution: 0.1\norigin: [-5.0, -5.0, 0.0]\n'
        'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n',
        encoding='utf-8')
    return yaml_path


def _document(map_yaml):
    document = new_field_route_document(
        'corn_field', 'corn_field_five_point_001', map_yaml)
    quality = {
        'samples': 30,
        'position_spread_m': 0.01,
        'yaw_spread_rad': 0.01,
        'max_position_stddev_m': 0.04,
        'max_yaw_stddev_rad': 0.04,
    }
    for index, step in enumerate(document['mission']['route_steps']):
        step['navigation_pose'] = {'x': -1.0 + index * 0.5, 'y': 0.0, 'yaw': 0.0}
        step['capture_quality'] = quality.copy()
        if step['role'] == 'inspect':
            step['tree_id'] = f'corn_{index:02d}'
            step['tree_offset_arm_base_m'] = [0.0, 1.2]
            step['tree_base_z_m'] = 0.0
            step['arm_spray_duration'] = ARM_SPRAY_DURATION_SEC
    return document


def test_v4_route_requires_exact_five_point_order_and_two_three_second_inspects(tmp_path):
    map_yaml = _map(tmp_path)
    document = _document(map_yaml)
    route_file = tmp_path / 'field_route.yaml'
    route_file.write_text(yaml.safe_dump(document), encoding='utf-8')

    steps = validate_field_route_document(load_field_route_document(route_file), map_yaml)

    assert tuple(step.point_id for step in steps) == ROUTE_POINT_IDS
    assert tuple(step.role for step in steps) == ROUTE_ROLES
    assert [step.arm_spray_duration for step in steps if step.role == 'inspect'] == [3.0, 3.0]


def test_v4_route_records_alicia_mount_yaw_and_accepts_legacy_missing_value(tmp_path):
    map_yaml = _map(tmp_path)
    document = _document(map_yaml)
    assert document['arm_base_mount']['yaw_rad'] == pytest.approx(
        ALICIA_ARM_BASE_YAW_RAD)

    document['arm_base_mount'].pop('yaw_rad')
    validate_field_route_document(document, map_yaml)

    document['arm_base_mount']['yaw_rad'] = 0.0
    with pytest.raises(ValueError, match='arm_base_mount does not match robot geometry'):
        validate_field_route_document(document, map_yaml)


def test_v4_route_rejects_role_order_duplicate_tree_and_duration_drift(tmp_path):
    map_yaml = _map(tmp_path)
    document = _document(map_yaml)
    document['mission']['route_steps'][0]['role'] = 'inspect'
    with pytest.raises(ValueError, match='wide_start'):
        route_steps(document)

    document = _document(map_yaml)
    document['mission']['route_steps'][2]['tree_id'] = 'corn_01'
    with pytest.raises(ValueError, match='duplicate inspect tree_id'):
        validate_field_route_document(document, map_yaml)

    document = _document(map_yaml)
    document['mission']['route_steps'][1]['arm_spray_duration'] = 2.9
    with pytest.raises(ValueError, match='must be 3.0'):
        route_steps(document)


def test_v4_default_validation_keeps_geometry_but_not_capture_quality_gates(tmp_path):
    map_yaml = _map(tmp_path)
    document = _document(map_yaml)
    for step in document['mission']['route_steps']:
        step.pop('capture_quality', None)
        step['navigation_pose']['x'] = 100.0

    validate_field_route_document(document, map_yaml)
    with pytest.raises(ValueError, match='capture_quality'):
        validate_field_route_document(
            document, map_yaml,
            require_capture_quality=True,
            require_free_space=True,
        )


def test_real_file_mode_keeps_shared_manual_route_untouched():
    manager = (PACKAGE / 'scripts' / 'field_route_manager.py').read_text(
        encoding='utf-8')
    orchestration = (PACKAGE / 'launch' / 'real_orchestration.launch.py').read_text(
        encoding='utf-8')

    assert "executable='field_route_manager.py'" in orchestration
    assert "executable='load_site_mission.py'" not in orchestration
    # The generic manager is started only by mission_mode:=qt.  File mode
    # retains this manager unchanged for its existing schema-v4 route.
    assert "package='wvcsc_mission_manager', executable='mission_manager'" in orchestration
    assert "' == 'file'" in orchestration
    assert "self._relay(\n                self._wide_channel, True" in manager
    assert "self._relay(\n                self._wide_channel, False" in manager
    assert 'self._command_all_off()' in manager
    assert 'ExecuteSpray.Result.OK' in manager
    assert 'ExecuteSpray.Result.INSPECTED_NO_DISEASE' in manager
    assert 'ExecuteSpray.Result.PARTIAL_SUCCESS' in manager
    assert 'ExecuteSpray.Result.OBSERVE_FAILED' in manager
    assert 'ExecuteSpray.Result.VISION_FAILED' in manager
    assert 'def _skip_current_step' in manager
    assert 'self._skipped_targets += 1' in manager
    assert "'/field_route/cancel'" in manager
    assert "MissionStatus.ARM_SPRAYING" in manager
    assert 'VERIFYING_INSPECT_STOP' in manager
    assert 'VERIFYING_FINISH_STOP' in manager
    assert 'vehicle stop verification' in manager


def test_field_manager_continues_after_relay_and_recoverable_step_failures():
    manager = (PACKAGE / 'scripts' / 'field_route_manager.py').read_text(
        encoding='utf-8')
    relay = manager.split('    def _relay(', 1)[1].split(
        '    def _command_all_off(', 1)[0]
    clients_ready = manager.split('    def _clients_ready(self):', 1)[1].split(
        '    def _tick(self):', 1)[0]
    docs = (PACKAGE.parent / 'docs' /
            'WVCSC_S2Z_UTB_ARM_实车导航验收指南.md').read_text(
                encoding='utf-8')

    assert 'def _relay_failure_continue' in relay
    assert '[FIELD_ROUTE][WARN][RELAY]' in relay
    assert 'continuation()' in relay
    assert 'self._fail(' not in relay
    assert '_relay_client' not in clients_ready
    assert 'skipped_targets' in manager
    assert '单点失败跳过、路线继续' in docs
    assert '[FIELD_ROUTE][WARN][RELAY]' in docs


def test_field_manager_uses_latched_mission_status_for_yolo():
    manager = (PACKAGE / 'scripts' / 'field_route_manager.py').read_text(
        encoding='utf-8')

    assert 'from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy' in manager
    assert 'mission_status_qos = QoSProfile(' in manager
    assert 'reliability=ReliabilityPolicy.RELIABLE' in manager
    assert 'durability=DurabilityPolicy.TRANSIENT_LOCAL' in manager
    assert "MissionStatus, '/mission/status', mission_status_qos" in manager


def test_field_manager_waits_for_full_nav2_and_localization_before_point_one():
    manager = (PACKAGE / 'scripts' / 'field_route_manager.py').read_text(
        encoding='utf-8')

    for node in (
            '/amcl', '/map_server', '/controller_server', '/planner_server',
            '/smoother_server', '/behavior_server', '/bt_navigator',
            '/waypoint_follower', '/velocity_smoother'):
        assert repr(node) in manager
    assert 'all required Nav2 lifecycle nodes are ACTIVE' in manager
    assert 'lookup_transform(' in manager
    assert 'self._map_frame, self._base_frame' in manager


def test_field_manager_enables_wide_spray_only_after_accepted_motion():
    manager = (PACKAGE / 'scripts' / 'field_route_manager.py').read_text(
        encoding='utf-8')
    start_navigation = manager.split('    def _start_navigation(self):', 1)[1].split(
        '    def _send_nav_goal(self):', 1)[0]

    assert 'self._relay(' not in start_navigation
    assert 'WAITING_FOR_NAV_MOTION' in manager
    assert 'wide_spray_motion_linear_threshold' in manager
    assert 'vehicle motion confirmed; enable wide spray' in manager
    assert 'vehicle did not begin moving before wide spray timeout' in manager


def test_current_field_routes_keep_positive_arm_y_and_record_mount_yaw():
    routes = sorted((PACKAGE / 'config' / 'real').glob(
        'mission_*/field_route_corn.yaml'))
    assert routes
    for route_path in routes:
        document = yaml.safe_load(route_path.read_text(encoding='utf-8'))
        assert document['arm_base_mount']['yaw_rad'] == pytest.approx(
            ALICIA_ARM_BASE_YAW_RAD)
        inspect_steps = [
            step for step in document['mission']['route_steps']
            if step['role'] == 'inspect'
        ]
        assert len(inspect_steps) == 2
        assert all(step['tree_offset_arm_base_m'][1] > 0.0
                   for step in inspect_steps)
