from dataclasses import dataclass
import math


def _value(node, name):
    return node.get_parameter(name).value


@dataclass(frozen=True)
class ServoRuntimeConfig:
    control_rate_hz: float
    default_timeout_sec: float
    stale_timeout_sec: float
    invalid_target_hold_sec: float
    min_confidence: float
    coarse_tolerance_px: float
    fine_tolerance_px: float
    control_resume_tolerance_px: float
    stable_duration_sec: float
    progress_window_sec: float
    min_progress_px: float
    desired_offset_u_px: float
    desired_offset_v_px: float
    aim_compensation_enabled: bool
    aim_range_source: str
    aim_range_min_m: float
    aim_range_max_m: float
    aim_nozzle_frame: str
    aim_min_forward_axis_z: float
    aim_image_margin_px: float
    max_linear_speed: float
    max_linear_acceleration: float
    near_target_speed_scale: float
    warning_speed_scale: float
    predict_lead_sec: float
    max_predict_horizon_sec: float
    command_mode: str
    max_angular_speed: float
    max_angular_acceleration: float

    @classmethod
    def from_node(cls, node):
        config = cls(
            control_rate_hz=float(_value(node, 'control_rate_hz')),
            default_timeout_sec=float(_value(node, 'default_timeout_sec')),
            stale_timeout_sec=float(_value(node, 'target_stale_timeout_sec')),
            invalid_target_hold_sec=float(_value(node, 'target_invalid_hold_sec')),
            min_confidence=float(_value(node, 'min_confidence')),
            coarse_tolerance_px=float(_value(node, 'coarse_tolerance_px')),
            fine_tolerance_px=float(_value(node, 'fine_tolerance_px')),
            control_resume_tolerance_px=float(
                _value(node, 'control_resume_tolerance_px')),
            stable_duration_sec=float(_value(node, 'stable_duration_sec')),
            progress_window_sec=float(_value(node, 'progress_window_sec')),
            min_progress_px=float(_value(node, 'min_progress_px')),
            desired_offset_u_px=float(_value(node, 'desired_offset_u_px')),
            desired_offset_v_px=float(_value(node, 'desired_offset_v_px')),
            aim_compensation_enabled=bool(
                _value(node, 'aim_compensation_enabled')),
            aim_range_source=str(_value(node, 'aim_range_source')),
            aim_range_min_m=float(_value(node, 'aim_range_min_m')),
            aim_range_max_m=float(_value(node, 'aim_range_max_m')),
            aim_nozzle_frame=str(_value(node, 'aim_nozzle_frame')).strip(),
            aim_min_forward_axis_z=float(
                _value(node, 'aim_min_forward_axis_z')),
            aim_image_margin_px=float(_value(node, 'aim_image_margin_px')),
            max_linear_speed=float(_value(node, 'max_linear_speed')),
            max_linear_acceleration=float(_value(node, 'max_linear_acceleration')),
            near_target_speed_scale=float(_value(node, 'near_target_speed_scale')),
            warning_speed_scale=float(_value(node, 'warning_speed_scale')),
            predict_lead_sec=float(_value(node, 'predict_lead_sec')),
            max_predict_horizon_sec=float(_value(node, 'max_predict_horizon_sec')),
            command_mode=str(_value(node, 'command_mode')),
            max_angular_speed=float(_value(node, 'max_angular_speed')),
            max_angular_acceleration=float(
                _value(node, 'max_angular_acceleration')),
        )
        if (config.control_rate_hz <= 0.0 or
                config.stable_duration_sec <= 0.0 or
                config.progress_window_sec <= 0.0 or
                config.min_progress_px <= 0.0 or
                config.stale_timeout_sec <= 0.0 or
                not 0.0 <= config.invalid_target_hold_sec <= config.stale_timeout_sec):
            raise ValueError(
                'control, convergence, progress or stale target timing is invalid')
        if (not math.isfinite(config.fine_tolerance_px)
                or not math.isfinite(config.control_resume_tolerance_px)
                or config.fine_tolerance_px <= 0.0
                or config.control_resume_tolerance_px < config.fine_tolerance_px):
            raise ValueError('alignment tolerance hysteresis is invalid')
        if not 0.0 < config.near_target_speed_scale <= 1.0:
            raise ValueError('near_target_speed_scale must be in (0, 1]')
        if config.command_mode not in {'linear_xy', 'angular_xy'}:
            raise ValueError('command_mode must be linear_xy or angular_xy')
        if config.aim_range_source != 'goal':
            raise ValueError('aim_range_source must be goal')
        aim_values = (
            config.aim_range_min_m,
            config.aim_range_max_m,
            config.aim_min_forward_axis_z,
            config.aim_image_margin_px,
            config.desired_offset_u_px,
            config.desired_offset_v_px,
        )
        if (not all(math.isfinite(value) for value in aim_values)
                or config.aim_range_min_m <= 0.0
                or config.aim_range_max_m <= config.aim_range_min_m
                or not 0.0 < config.aim_min_forward_axis_z <= 1.0
                or config.aim_image_margin_px < 0.0
                or (config.aim_compensation_enabled
                    and not config.aim_nozzle_frame)):
            raise ValueError('aim compensation parameters are invalid')
        if (not math.isfinite(config.max_angular_speed)
                or not math.isfinite(config.max_angular_acceleration)
                or config.max_angular_speed <= 0.0
                or config.max_angular_acceleration <= 0.0):
            raise ValueError('angular command limits are invalid')
        return config
