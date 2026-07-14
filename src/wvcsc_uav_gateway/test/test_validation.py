from pathlib import Path

import pytest

from wvcsc_uav_gateway.validation import load_and_validate


def _write(tmp_path, text):
    path = Path(tmp_path) / 'mission.yaml'
    path.write_text(text, encoding='utf-8')
    return str(path)


VALID = '''
mission:
  mission_id: demo
  frame_id: map
  source_mode: mock
  trees:
    - tree_id: tree_1
      confidence: 0.9
      position: {x: 3.0, y: 2.0}
      spray_side: left
      spray_duration: 2.0
'''


def test_accepts_valid_mission(tmp_path):
    mission = load_and_validate(_write(tmp_path, VALID))
    assert mission['mission_id'] == 'demo'
    assert mission['trees'][0]['position'] == {'x': 3.0, 'y': 2.0, 'z': 0.0}


@pytest.mark.parametrize('old,new', [
    ('confidence: 0.9', 'confidence: .nan'),
    ('spray_side: left', 'spray_side: ahead'),
    ('spray_duration: 2.0', 'spray_duration: .inf'),
])
def test_rejects_invalid_target(tmp_path, old, new):
    text = VALID.replace(old, new, 1)
    with pytest.raises(ValueError):
        load_and_validate(_write(tmp_path, text))


def test_rejects_duplicate_tree_id(tmp_path):
    duplicate = '''
    - tree_id: tree_1
      confidence: 0.8
      position: {x: 4.0, y: -2.0}
      spray_side: right
      spray_duration: 2.0
'''
    with pytest.raises(ValueError):
        load_and_validate(_write(tmp_path, VALID + duplicate))
