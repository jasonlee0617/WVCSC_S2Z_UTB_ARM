import math

import yaml


def load_and_validate(
        path, confidence_threshold=0.5, min_duration=0.2,
        max_duration=10.0, max_abs_coordinate=50.0):
    with open(path, encoding='utf-8') as stream:
        data = yaml.safe_load(stream)
    return validate_mission(
        data, confidence_threshold, min_duration, max_duration,
        max_abs_coordinate, expected_source_mode='mock')


def validate_mission(
        data, confidence_threshold=0.5, min_duration=0.2,
        max_duration=10.0, max_abs_coordinate=50.0,
        expected_source_mode=None):
    mission = data.get('mission') if isinstance(data, dict) else None
    if not isinstance(mission, dict):
        raise ValueError('missing mission mapping')

    mission_id = str(mission.get('mission_id', '')).strip()
    if not mission_id:
        raise ValueError('mission_id is required')
    frame_id = str(mission.get('frame_id', '')).strip()
    if frame_id != 'map':
        raise ValueError('mission frame_id must be map')
    source_mode = str(mission.get('source_mode', 'mock')).strip().lower()
    if expected_source_mode and source_mode != expected_source_mode:
        raise ValueError(
            f'gateway source_mode must be {expected_source_mode}')
    if source_mode not in ('mock', 'replay', 'live'):
        raise ValueError('source_mode must be mock, replay or live')

    trees = mission.get('trees')
    if not isinstance(trees, list) or not trees:
        raise ValueError('mission must contain at least one tree')

    seen = set()
    normalized = []
    for item in trees:
        if not isinstance(item, dict):
            raise ValueError('each tree must be a mapping')
        tree_id = str(item.get('tree_id', '')).strip()
        if not tree_id or tree_id in seen:
            raise ValueError(f'invalid or duplicate tree_id: {tree_id!r}')
        seen.add(tree_id)

        confidence = _finite(item.get('confidence'), 'confidence')
        if not confidence_threshold <= confidence <= 1.0:
            raise ValueError(f'{tree_id}: confidence out of range')
        side = str(item.get('spray_side', '')).strip().lower()
        if side not in ('left', 'right'):
            raise ValueError(f'{tree_id}: spray_side must be left or right')
        duration = _finite(item.get('spray_duration'), 'spray_duration')
        if not min_duration <= duration <= max_duration:
            raise ValueError(f'{tree_id}: spray_duration out of range')

        position = item.get('position')
        if not isinstance(position, dict):
            raise ValueError(f'{tree_id}: position is required')
        xyz = {
            axis: _finite(position.get(axis, 0.0), f'position.{axis}')
            for axis in ('x', 'y', 'z')
        }
        if abs(xyz['x']) > max_abs_coordinate or abs(xyz['y']) > max_abs_coordinate:
            raise ValueError(f'{tree_id}: position out of bounds')
        normalized.append({
            'tree_id': tree_id,
            'confidence': confidence,
            'position': xyz,
            'spray_side': side,
            'spray_duration': duration,
            'evidence_uri': str(item.get('evidence_uri', '')),
        })

    return {
        'mission_id': mission_id,
        'source_mode': source_mode,
        'frame_id': frame_id,
        'publish_delay_sec': max(
            0.0, _finite(mission.get('publish_delay_sec', 0.0), 'publish_delay_sec')),
        'trees': normalized,
    }


def load_and_validate_replay(
        path, confidence_threshold=0.5, min_duration=0.2,
        max_duration=10.0, max_abs_coordinate=50.0):
    with open(path, encoding='utf-8') as stream:
        data = yaml.safe_load(stream)
    replay = data.get('replay') if isinstance(data, dict) else None
    if not isinstance(replay, dict):
        raise ValueError('missing replay mapping')
    playback_rate = _finite(replay.get('playback_rate', 1.0), 'playback_rate')
    loop_delay = _finite(replay.get('loop_delay_sec', 1.0), 'loop_delay_sec')
    if playback_rate <= 0.0:
        raise ValueError('playback_rate must be positive')
    if loop_delay < 0.0:
        raise ValueError('loop_delay_sec cannot be negative')
    events = replay.get('events')
    if not isinstance(events, list) or not events:
        raise ValueError('replay must contain at least one event')

    normalized = []
    mission_ids = set()
    previous_at = -1.0
    for event in events:
        if not isinstance(event, dict):
            raise ValueError('each replay event must be a mapping')
        at_sec = _finite(event.get('at_sec'), 'event.at_sec')
        if at_sec < 0.0 or at_sec < previous_at:
            raise ValueError('replay event times must be non-negative and ordered')
        previous_at = at_sec
        mission = validate_mission(
            {'mission': event.get('mission')}, confidence_threshold,
            min_duration, max_duration, max_abs_coordinate,
            expected_source_mode='replay')
        if mission['mission_id'] in mission_ids:
            raise ValueError('replay mission_id values must be unique')
        mission_ids.add(mission['mission_id'])
        normalized.append({'at_sec': at_sec, 'mission': mission})

    return {
        'playback_rate': playback_rate,
        'loop': bool(replay.get('loop', False)),
        'loop_delay_sec': loop_delay,
        'events': normalized,
    }


def _finite(value, name):
    try:
        number = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f'{name} must be numeric') from error
    if not math.isfinite(number):
        raise ValueError(f'{name} must be finite')
    return number
