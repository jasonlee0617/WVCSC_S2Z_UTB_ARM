import math

import pytest

from wvcsc_bringup.site_mission import (
    atomic_write_site,
    circular_mean,
    load_site_document,
    map_hashes,
    new_site_document,
    pose_sample_statistics,
    tree_hint_from_offset,
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
    docking = (0.0, 0.0, 0.0)
    hint = tree_hint_from_offset(docking, 0.0, 1.2)
    document['mission']['targets'].append({
        'target_id': 'corn_01',
        'docking_pose': {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
        'tree_hint': {'x': hint[0], 'y': hint[1], 'z': hint[2]},
        'measured_tree_offset': {'forward_m': 0.0, 'left_m': 1.2},
        'spray_side': 'left',
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


def test_tree_offset_transform_uses_base_forward_and_left_axes():
    x, y, z = tree_hint_from_offset((2.0, 3.0, math.pi / 2.0), 1.0, 2.0)
    assert x == pytest.approx(0.0)
    assert y == pytest.approx(4.0)
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


def test_side_hint_and_quality_mismatches_are_rejected(tmp_path):
    map_yaml, _image = _map(tmp_path)
    document = _document(map_yaml)
    document['mission']['targets'][0]['spray_side'] = 'right'
    with pytest.raises(ValueError, match='spray_side conflicts'):
        validate_site_document(document, map_yaml)

    document = _document(map_yaml)
    document['mission']['targets'][0]['capture_quality']['samples'] = 29
    with pytest.raises(ValueError, match='at least 30'):
        validate_site_document(document, map_yaml)


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
