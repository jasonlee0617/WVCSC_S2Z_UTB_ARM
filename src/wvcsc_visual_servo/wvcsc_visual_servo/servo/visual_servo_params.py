from dataclasses import dataclass


def _value(node, name):
    return node.get_parameter(name).value


@dataclass(frozen=True)
class ServoRuntimeConfig:
    control_rate_hz: float
    default_timeout_sec: float
    stale_timeout_sec: float
    min_confidence: float
    coarse_tolerance_px: float
    fine_tolerance_px: float
    stable_frames: int
    desired_offset_u_px: float
    desired_offset_v_px: float
    max_linear_speed: float
    max_linear_acceleration: float
    near_target_speed_scale: float
    warning_speed_scale: float
    command_sign_x: float
    command_sign_y: float
    predict_lead_sec: float
    max_predict_horizon_sec: float

    @classmethod
    def from_node(cls, node):
        config = cls(
            control_rate_hz=float(_value(node, 'control_rate_hz')),
            default_timeout_sec=float(_value(node, 'default_timeout_sec')),
            stale_timeout_sec=float(_value(node, 'target_stale_timeout_sec')),
            min_confidence=float(_value(node, 'min_confidence')),
            coarse_tolerance_px=float(_value(node, 'coarse_tolerance_px')),
            fine_tolerance_px=float(_value(node, 'fine_tolerance_px')),
            stable_frames=int(_value(node, 'stable_frames')),
            desired_offset_u_px=float(_value(node, 'desired_offset_u_px')),
            desired_offset_v_px=float(_value(node, 'desired_offset_v_px')),
            max_linear_speed=float(_value(node, 'max_linear_speed')),
            max_linear_acceleration=float(_value(node, 'max_linear_acceleration')),
            near_target_speed_scale=float(_value(node, 'near_target_speed_scale')),
            warning_speed_scale=float(_value(node, 'warning_speed_scale')),
            command_sign_x=float(_value(node, 'command_sign_x')),
            command_sign_y=float(_value(node, 'command_sign_y')),
            predict_lead_sec=float(_value(node, 'predict_lead_sec')),
            max_predict_horizon_sec=float(_value(node, 'max_predict_horizon_sec')),
        )
        if config.control_rate_hz <= 0.0 or config.stable_frames <= 0:
            raise ValueError('control_rate_hz and stable_frames must be positive')
        if not 0.0 < config.near_target_speed_scale <= 1.0:
            raise ValueError('near_target_speed_scale must be in (0, 1]')
        return config
