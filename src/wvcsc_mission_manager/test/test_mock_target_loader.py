from builtin_interfaces.msg import Time
import pytest
import yaml

from wvcsc_mission_manager.mock_target_loader import (
    build_request,
    load_mock_document,
)


def _document():
    return {
        'mission': {
            'mission_id': 'mock_loader_test',
            'frame_id': 'map',
            'load_delay_sec': 0.0,
            'return_home_after_finish': True,
            'home_pose': {'x': 0.2, 'y': -0.1, 'yaw': 0.3},
            'trees': [{
                'tree_id': 'tree_01',
                'confidence': 0.9,
                'position': {'x': 3.0, 'y': 2.0, 'z': 0.0},
                'spray_duration': 2.0,
                'evidence_uri': 'mock://tree_01',
            }],
        },
    }


def test_mock_target_loader_builds_explicit_tree_hint_and_computed_docking(tmp_path):
    path = tmp_path / 'mock_targets.yaml'
    path.write_text(yaml.safe_dump(_document()), encoding='utf-8')

    document = load_mock_document(path)
    request = build_request(document, Time(sec=12, nanosec=34))

    assert request.header.frame_id == 'map'
    assert request.return_home_after_finish
    assert request.home_pose.position.x == pytest.approx(0.2)
    assert request.home_pose.position.y == pytest.approx(-0.1)
    assert len(request.targets) == 1
    target = request.targets[0]
    assert target.target_id == 'tree_01'
    assert target.confidence == pytest.approx(0.9)
    assert target.evidence_uri == 'mock://tree_01'
    assert target.use_explicit_tree_hint
    assert target.compute_docking_pose
    assert (target.tree_hint.x, target.tree_hint.y, target.tree_hint.z) == (
        pytest.approx(3.0), pytest.approx(2.0), pytest.approx(0.0))


def test_mock_target_loader_rejects_invalid_target_confidence(tmp_path):
    document = _document()
    document['mission']['trees'][0]['confidence'] = 0.0
    path = tmp_path / 'mock_targets.yaml'
    path.write_text(yaml.safe_dump(document), encoding='utf-8')

    with pytest.raises(ValueError, match='confidence'):
        load_mock_document(path)
