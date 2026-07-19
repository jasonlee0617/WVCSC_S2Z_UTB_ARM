import json


DEBUG_DEFAULTS = {
    'event': '',
    'mission_id': '',
    'tree_id': '',
    'target_id': '',
    'elapsed_sec': 0.0,
    'camera_ready': False,
    'target_valid': False,
    'target_age_sec': -1.0,
    'target_unavailable_sec': 0.0,
    'confidence': 0.0,
    'error_u_px': 0.0,
    'error_v_px': 0.0,
    'last_valid_error_u_px': 0.0,
    'last_valid_error_v_px': 0.0,
    'stable_frames': 0,
    'stable_duration_sec': 0.0,
    'progress_stalled': False,
    # Preserve the original linear fields for bag compatibility and add the
    # angular command actually sent by the eye-in-hand profile.
    'command_mode': 'linear_xy',
    'command_x_mps': 0.0,
    'command_y_mps': 0.0,
    'command_angular_x_rps': 0.0,
    'command_angular_y_rps': 0.0,
    'control_dt_sec': 0.0,
    'servo_status': 0,
    'servo_status_text': 'NO_WARNING',
    'joint_positions': [],
    'result_code': -1,
    'message': '',
}


def debug_json(**values):
    payload = dict(DEBUG_DEFAULTS)
    payload.update({key: values[key] for key in payload if key in values})
    return json.dumps(payload, ensure_ascii=False, separators=(',', ':'))


def debug_publish_due(now, last_publish, rate_hz, force=False):
    return force or last_publish is None or now - last_publish >= 1.0 / rate_hz
