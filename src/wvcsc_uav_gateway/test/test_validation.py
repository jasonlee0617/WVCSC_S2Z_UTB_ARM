from pathlib import Path

import pytest

from wvcsc_uav_gateway.validation import (
    load_and_validate,
    load_and_validate_replay,
)


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


def test_accepts_ordered_replay_events(tmp_path):
    replay = '''
replay:
  playback_rate: 2.0
  loop: false
  events:
    - at_sec: 0.1
      mission:
        mission_id: replay_1
        frame_id: map
        source_mode: replay
        trees:
          - tree_id: tree_1
            confidence: 0.9
            position: {x: 3.0, y: 2.0}
            spray_side: left
            spray_duration: 2.0
'''
    config = load_and_validate_replay(_write(tmp_path, replay))
    assert config['playback_rate'] == 2.0
    assert config['events'][0]['mission']['source_mode'] == 'replay'


@pytest.mark.parametrize('replacement', [
    'at_sec: -0.1',
    'source_mode: live',
    'playback_rate: 0.0',
])
def test_rejects_invalid_replay_config(tmp_path, replacement):
    replay = '''
replay:
  playback_rate: 1.0
  events:
    - at_sec: 0.1
      mission:
        mission_id: replay_1
        frame_id: map
        source_mode: replay
        trees:
          - tree_id: tree_1
            confidence: 0.9
            position: {x: 3.0, y: 2.0}
            spray_side: left
            spray_duration: 2.0
'''
    if replacement.startswith('playback_rate'):
        invalid = replay.replace('playback_rate: 1.0', replacement)
    elif replacement.startswith('source_mode'):
        invalid = replay.replace('source_mode: replay', replacement)
    else:
        invalid = replay.replace('at_sec: 0.1', replacement)
    with pytest.raises(ValueError):
        load_and_validate_replay(_write(tmp_path, invalid))
