import math

import pytest

from wvcsc_bringup.site_mission import (
    atomic_write_site,
    circular_mean,
    load_site_document,
    map_hashes,
    migrate_site_document,
    new_site_document,
    pose_sample_statistics,
    tree_hint_from_arm_base_offset,
    validate_site_document,
)


def _map(tmp_path):
    image = tmp_path / 'map.pgm'
    image.write_bytes(b'P5\n100 100\n255\n' + bytes([254]) * 10000)
    yaml_path = tmp_path / 'map.yaml'
    yaml_path.write_text(
        'image: map.pgm\nresolution: 0.1\norigin: [-5.0, -5.0, 0.0]\n'
        'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n',
        encoding='utf-8')
    return yaml_path, image


def _document(map_yaml):
    document = new_site_document('corn_site', 'corn_measured_001', map_yaml)
    document['mission']['home_pose'] = {'x': -2.0, 'y': -2.0, 'yaw': 0.0}
    document['mission']['targets'].append({
        'target_id': 'corn_01',
        'docking_pose': {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
        'tree_offset_arm_base_m': {
            'reference': 'alicia_base_link_xy',
            'x_m': 0.0,
            'y_m': 1.2,
        },
        'tree_base_z_m': 0.0,
        'spray_duration': 5.0,
        'capture_quality': {
            'samples': 30,
            'position_spread_m': 0.01,
            'yaw_spread_rad': 0.01,
            'max_position_stddev_m': 0.04,
            'max_yaw_stddev_rad': 0.04,
        },
    })
    return document


def test_tree_offset_transform_uses_arm_base_origin_and_signed_xy():
    x, y, z = tree_hint_from_arm_base_offset(
        (2.0, 3.0, math.pi / 2.0), 1.0, 2.0)
    assert x == pytest.approx(0.0)
    assert y == pytest.approx(3.6)
    assert z == 0.0


def test_circular_pose_statistics_are_stable_across_pi_boundary():
    samples = [(1.0, 2.0, math.pi - 0.01),
               (1.01, 1.99, -math.pi + 0.01)]
    x, y, yaw, position_spread, yaw_spread = pose_sample_statistics(samples)
    assert x == pytest.approx(1.005)
    assert y == pytest.approx(1.995)
    assert abs(abs(yaw) - math.pi) < 1e-6
    assert position_spread < 0.01
    assert yaw_spread == pytest.approx(0.01)
    assert abs(abs(circular_mean([math.pi - 0.01, -math.pi + 0.01])) -
               math.pi) < 1e-6


def test_site_document_is_bound_to_map_and_requires_capture_quality(tmp_path):
    map_yaml, image = _map(tmp_path)
    document = _document(map_yaml)
    validate_site_document(document, map_yaml)

    image.write_bytes(b'P5\n100 100\n255\n' + bytes([253]) * 10000)
    with pytest.raises(ValueError, match='image SHA256'):
        validate_site_document(document, map_yaml)


def test_tree_offset_and_quality_mismatches_are_rejected(tmp_path):
    map_yaml, _image = _map(tmp_path)
    document = _document(map_yaml)
    document['mission']['targets'][0]['capture_quality']['samples'] = 29
    with pytest.raises(ValueError, match='at least 30'):
        validate_site_document(document, map_yaml)

    document = _document(map_yaml)
    document['mission']['targets'][0]['tree_offset_arm_base_m']['x_m'] = 0.21
    with pytest.raises(ValueError, match='arm-base X error'):
        validate_site_document(document, map_yaml)


def test_schema_v1_is_rejected_instead_of_silently_reinterpreted(tmp_path):
    path = tmp_path / 'site.yaml'
    path.write_text('schema_version: 1\nsite_id: old\n', encoding='utf-8')

    with pytest.raises(ValueError, match='schema_version must be 3'):
        load_site_document(path)


def test_schema_v2_is_migrated_only_from_recorded_numeric_offsets(tmp_path):
    map_yaml, _image = _map(tmp_path)
    document = _document(map_yaml)
    document['schema_version'] = 2
    document['arm_base_mount'] = {'forward_m': -0.40, 'left_m': 0.0}
    target = document['mission']['targets'][0]
    hint = tree_hint_from_arm_base_offset((0.0, 0.0, 0.0), 0.0, 1.2)
    target['tree_hint'] = {'x': hint[0], 'y': hint[1], 'z': hint[2]}
    target['measured_tree_offset'] = {
        'reference': 'arm_base_vehicle_axes',
        'forward_m': 0.0,
        'left_m': 1.2,
    }
    target['spray_side'] = 'left'
    target.pop('tree_offset_arm_base_m')
    target.pop('tree_base_z_m')

    converted = migrate_site_document(document, map_yaml)
    assert converted['schema_version'] == 3
    assert converted['mission']['targets'][0]['tree_offset_arm_base_m'] == {
        'reference': 'alicia_base_link_xy', 'x_m': 0.0, 'y_m': 1.2}
    assert 'spray_side' not in converted['mission']['targets'][0]


def test_atomic_site_write_round_trips_and_keeps_backup(tmp_path):
    map_yaml, _image = _map(tmp_path)
    path = tmp_path / 'site.yaml'
    first = _document(map_yaml)
    atomic_write_site(path, first)
    assert load_site_document(path) == first

    second = _document(map_yaml)
    second['mission']['mission_id'] = 'updated'
    atomic_write_site(path, second)
    assert load_site_document(path)['mission']['mission_id'] == 'updated'
    assert path.with_suffix('.yaml.bak').is_file()
    assert map_hashes(map_yaml) == first['map']
