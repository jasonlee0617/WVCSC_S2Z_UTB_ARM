"""喷洒任务参数声明、解析与校验。

目标发现参数把“观察位总检测时间 n”和“目标出现率窗口 m”明确分开。配置加载时
强制检查 ``0 < m <= n``、出现率范围和最小有效帧数，避免无效门控进入实机任务。
"""

import math
from dataclasses import dataclass
from types import MappingProxyType
from typing import Mapping


DEFAULT_JOINT_PRESETS_DEG = {
    'center': (95.3, -136.9, -71.0, 7.7, 57.3, -4.4),
    'fan_left': (52.2, -131.7, -55.4, -58.9, 76.5, 18.2),
    'fan_right': (118.5, -129.4, -55.8, 47.6, 66.2, -17.1),
    'right_center': (-105.4, -127.8, -50.5, -15.4, 71.2, -4.9),
    'right_fan_left': (-139.8, -128.6, -57.3, -70.1, 79.7, 13.0),
    'right_fan_right': (-70.8, -126.8, -50.6, 32.2, 69.8, -12.1),
}


def parameter_defaults():
    """为每个喷洒节点实例返回一份独立的 ROS 参数默认值。"""
    return {
        'home_pose': [0.0] * 6,
        'min_spray_duration': 0.2,
        'max_spray_duration': 10.0,
        'post_spray_home_delay_sec': 2.0,
        'vision_action_name': '/vision/align_target',
        'aim_service_name': '/vision/compute_spray_aim',
        'aim_service_timeout_sec': 2.0,
        'vision_timeout_sec': 8.0,
        'spray_action_name': '/spray/execute',
        'downstream_server_timeout_sec': 2.0,
        'downstream_result_margin_sec': 2.0,
        'diseased_target_detection_topic': (
            '/vision/diseased_target_detections'),
        'vision_target_topic': '/vision/target',
        'selected_target_topic': '/vision/selected_target_id',
        'inference_mode_topic': '/vision/inference_mode',
        'motion_locked_topic': '/motion_control/locked',
        'max_targets_per_tree': 0,
        'spray_on_alignment_failure': False,
        'observation_mode': 'ik',
        'joint_preset_center_deg': list(DEFAULT_JOINT_PRESETS_DEG['center']),
        'joint_preset_fan_left_deg': list(
            DEFAULT_JOINT_PRESETS_DEG['fan_left']),
        'joint_preset_fan_right_deg': list(
            DEFAULT_JOINT_PRESETS_DEG['fan_right']),
        'joint_preset_right_center_deg': list(
            DEFAULT_JOINT_PRESETS_DEG['right_center']),
        'joint_preset_right_fan_left_deg': list(
            DEFAULT_JOINT_PRESETS_DEG['right_fan_left']),
        'joint_preset_right_fan_right_deg': list(
            DEFAULT_JOINT_PRESETS_DEG['right_fan_right']),
        'joint_preset_side_epsilon_m': 0.05,
        'confirmation_frames': 3,
        'detection_timeout_sec': 2.0,
        'fruit_collection_settle_sec': 1.00,
        'view_detection_duration_sec': 5.0,
        'target_presence_window_sec': 1.0,
        'target_presence_ratio': 0.50,
        'target_presence_min_frames': 5,
        'max_alignment_attempts': 2,
        'target_recenter_trigger_px': 48.0,
        'visual_servo_entry_max_error_px': 48.0,
        'cross_view_reassociation_max_distance_px': 320.0,
        # RGB-only cross-view association uses the recorded tree plane.  Keep
        # this small so two nearby leaves are never merged merely to avoid a
        # duplicate spray; ambiguous cases remain unresolved.
        'cross_view_target_distance_m': 0.08,
        'target_recenter_max_angle_deg': 20.0,
        'target_recenter_max_total_angle_deg': 30.0,
        'target_recenter_refine_goal_px': 8.0,
        'target_recenter_max_iterations': 8,
        'target_recenter_residual_candidates_px': [
            3.0, 8.0, 12.0, 16.0, 24.0, 32.0, 40.0, 64.0, 96.0,
            128.0, 160.0, 240.0, 320.0, 1.0, 0.0],
        'target_recenter_position_tolerance_m': 0.002,
        'target_recenter_orientation_tolerance_rad': 0.002,
        'target_post_recenter_stable_sec': 0.20,
        'target_post_recenter_max_drift_px': 4.0,
        'target_post_recenter_max_gap_sec': 0.20,
        'target_post_recenter_min_confidence': 0.30,
        'processed_iou_threshold': 0.30,
        'processed_center_distance_px': 18.0,
        'image_width': 640,
        'image_height': 480,
        'base_frame': 'alicia_base_link',
        'camera_frame': 'camera_color_optical_frame',
        'camera_info_topic': '/camera/color/camera_info',
        'joint_state_topic': '/joint_states',
        'robot_description': '',
        'observation_input_timeout_sec': 2.0,
        'observation_search_timeout_sec': 8.0,
        'observation_max_plans': 8,
        'observation_camera_reach_min_m': 0.20,
        'observation_camera_reach_max_m': 0.40,
        'observation_camera_reach_step_m': 0.10,
        'camera_height_min_m': 0.20,
        'camera_height_max_m': 0.40,
        'camera_height_step_m': 0.10,
        'observation_azimuth_offsets_deg': [0.0, -12.0, 12.0],
        'observation_center_height_m': 1.30,
        'spray_nozzle_frame': 'spray_nozzle_link',
        'observation_nozzle_plane_min_m': 0.20,
        'observation_nozzle_plane_max_m': 2.00,
        'observation_preferred_nozzle_plane_distance_m': 1.00,
        'observation_nozzle_plane_tolerance_m': 0.05,
        'observation_max_condition_number': 16.5,
        'observation_min_joint_margin_rad': 0.22,
        'observation_preferred_joint_margin_rad': 0.35,
        'observation_position_tolerance_m': 0.01,
        'observation_orientation_tolerance_rad': 0.01,
    }


def declare_spray_parameters(node):
    """在指定 ROS 节点上声明全部喷洒任务参数。"""
    for name, default in parameter_defaults().items():
        node.declare_parameter(name, default)


def _value(node, name):
    """读取指定 ROS 参数的原始值。"""
    return node.get_parameter(name).value


def joint_parameter(node, name):
    """读取并校验一个包含六个有限数值的关节参数。"""
    values = tuple(float(value) for value in _value(node, name))
    if len(values) != 6 or not all(math.isfinite(value) for value in values):
        raise ValueError(f'{name} must contain six finite joint positions')
    return values


def observation_parameters(node):
    """读取并校验动态 IK 观察位生成参数。"""
    values = {
        'camera_reach_min_m': float(
            _value(node, 'observation_camera_reach_min_m')),
        'camera_reach_max_m': float(
            _value(node, 'observation_camera_reach_max_m')),
        'camera_reach_step_m': float(
            _value(node, 'observation_camera_reach_step_m')),
        'camera_height_min_m': float(_value(node, 'camera_height_min_m')),
        'camera_height_max_m': float(_value(node, 'camera_height_max_m')),
        'camera_height_step_m': float(_value(node, 'camera_height_step_m')),
        'azimuth_offsets_deg': tuple(
            float(value)
            for value in _value(node, 'observation_azimuth_offsets_deg')),
        'center_height_m': float(
            _value(node, 'observation_center_height_m')),
        'nozzle_frame': str(_value(node, 'spray_nozzle_frame')).strip(),
        'nozzle_plane_min_m': float(
            _value(node, 'observation_nozzle_plane_min_m')),
        'nozzle_plane_max_m': float(
            _value(node, 'observation_nozzle_plane_max_m')),
        'preferred_nozzle_plane_distance_m': float(_value(
            node, 'observation_preferred_nozzle_plane_distance_m')),
        'nozzle_plane_tolerance_m': float(_value(
            node, 'observation_nozzle_plane_tolerance_m')),
        'max_condition_number': float(
            _value(node, 'observation_max_condition_number')),
        'min_joint_margin_rad': float(
            _value(node, 'observation_min_joint_margin_rad')),
        'preferred_joint_margin_rad': float(
            _value(node, 'observation_preferred_joint_margin_rad')),
        'position_tolerance_m': float(
            _value(node, 'observation_position_tolerance_m')),
        'orientation_tolerance_rad': float(
            _value(node, 'observation_orientation_tolerance_rad')),
    }
    positive = (
        'camera_reach_min_m', 'camera_reach_max_m', 'camera_reach_step_m',
        'camera_height_min_m', 'camera_height_max_m',
        'camera_height_step_m', 'center_height_m', 'nozzle_plane_min_m',
        'nozzle_plane_max_m', 'preferred_nozzle_plane_distance_m',
        'nozzle_plane_tolerance_m', 'max_condition_number',
        'min_joint_margin_rad', 'preferred_joint_margin_rad',
        'position_tolerance_m', 'orientation_tolerance_rad')
    if (not all(math.isfinite(values[name]) and values[name] > 0.0
                for name in positive) or
            values['camera_reach_min_m'] > values['camera_reach_max_m'] or
            values['camera_height_min_m'] > values['camera_height_max_m'] or
            values['nozzle_plane_min_m'] > values['nozzle_plane_max_m'] or
            not values['nozzle_plane_min_m'] <=
            values['preferred_nozzle_plane_distance_m'] <=
            values['nozzle_plane_max_m'] or
            values['preferred_joint_margin_rad'] <
            values['min_joint_margin_rad'] or
            not values['nozzle_frame'] or
            not values['azimuth_offsets_deg'] or
            not all(math.isfinite(value) for value in values[
                'azimuth_offsets_deg'])):
        raise ValueError('observation search parameters are invalid')
    return values


def target_recenter_parameters(node):
    """读取并校验目标粗重心和视觉伺服交接参数。"""
    values = {
        'trigger_px': float(_value(node, 'target_recenter_trigger_px')),
        'servo_entry_px': float(
            _value(node, 'visual_servo_entry_max_error_px')),
        'max_angle_deg': float(
            _value(node, 'target_recenter_max_angle_deg')),
        'max_total_angle_deg': float(
            _value(node, 'target_recenter_max_total_angle_deg')),
        'refine_goal_px': float(
            _value(node, 'target_recenter_refine_goal_px')),
        'max_iterations': int(
            _value(node, 'target_recenter_max_iterations')),
        'residual_candidates_px': tuple(dict.fromkeys(
            float(value)
            for value in _value(node, 'target_recenter_residual_candidates_px'))),
        'position_tolerance_m': float(
            _value(node, 'target_recenter_position_tolerance_m')),
        'orientation_tolerance_rad': float(
            _value(node, 'target_recenter_orientation_tolerance_rad')),
        'post_stable_sec': float(
            _value(node, 'target_post_recenter_stable_sec')),
        'post_max_drift_px': float(
            _value(node, 'target_post_recenter_max_drift_px')),
        'post_max_gap_sec': float(
            _value(node, 'target_post_recenter_max_gap_sec')),
        'post_min_confidence': float(
            _value(node, 'target_post_recenter_min_confidence')),
    }
    scalar_values = tuple(
        value for name, value in values.items()
        if name != 'residual_candidates_px')
    if (not all(math.isfinite(value) for value in scalar_values) or
            values['trigger_px'] <= 0.0 or
            values['max_angle_deg'] <= 0.0 or
            values['max_angle_deg'] > 180.0 or
            values['max_total_angle_deg'] < values['max_angle_deg'] or
            values['max_total_angle_deg'] > 180.0 or
            values['servo_entry_px'] > values['trigger_px'] or
            values['refine_goal_px'] <= 0.0 or
            values['refine_goal_px'] > values['trigger_px'] or
            values['position_tolerance_m'] <= 0.0 or
            values['orientation_tolerance_rad'] <= 0.0 or
            not 1 <= values['max_iterations'] <= 8 or
            not values['residual_candidates_px'] or
            any(not math.isfinite(value) or value < 0.0 or value > 4096.0
                for value in values['residual_candidates_px']) or
            values['post_stable_sec'] <= 0.0 or
            values['post_max_drift_px'] <= 0.0 or
            values['post_max_gap_sec'] <= 0.0 or
            values['post_max_gap_sec'] > values['post_stable_sec'] or
            not 0.0 <= values['post_min_confidence'] <= 1.0):
        raise ValueError('target recenter parameters are invalid')
    return values


def joint_preset_parameters(node):
    """读取观察模式及左右两侧固定关节预设。"""
    mode = str(_value(node, 'observation_mode')).strip().lower()
    if mode not in {'ik', 'joint_presets'}:
        raise ValueError('observation_mode must be ik or joint_presets')
    presets_by_side = {}
    definitions_by_side = {
        'left': (
            ('center', 'joint_preset_center_deg'),
            ('fan_left', 'joint_preset_fan_left_deg'),
            ('fan_right', 'joint_preset_fan_right_deg'),
        ),
        'right': (
            ('center', 'joint_preset_right_center_deg'),
            ('fan_left', 'joint_preset_right_fan_left_deg'),
            ('fan_right', 'joint_preset_right_fan_right_deg'),
        ),
    }
    for side, definitions in definitions_by_side.items():
        presets = []
        for name, parameter in definitions:
            try:
                degrees = tuple(
                    float(value) for value in _value(node, parameter))
            except (TypeError, ValueError) as error:
                raise ValueError(
                    f'{parameter} must contain six finite degrees') from error
            if (len(degrees) != 6 or
                    not all(math.isfinite(value) for value in degrees)):
                raise ValueError(
                    f'{parameter} must contain six finite degrees')
            presets.append((
                name, tuple(math.radians(value) for value in degrees)))
        presets_by_side[side] = tuple(presets)
    return mode, MappingProxyType(presets_by_side)


@dataclass(frozen=True)
class SprayConfig:
    """Immutable runtime snapshot of all spray-task ROS parameters."""

    home: tuple
    min_duration: float
    max_duration: float
    post_spray_home_delay: float
    vision_timeout: float
    downstream_server_timeout: float
    downstream_margin: float
    camera_frame: str
    base_frame: str
    spray_on_alignment_failure: bool
    observation_mode: str
    joint_preset_positions: Mapping
    joint_preset_side_epsilon_m: float
    observation: Mapping
    recenter: Mapping
    max_alignment_attempts: int
    max_targets_per_tree: int
    confirmation_frames: int
    detection_timeout: float
    fruit_collection_settle: float
    view_detection_duration: float
    target_presence_window: float
    target_presence_ratio: float
    target_presence_min_frames: int
    processed_iou_threshold: float
    processed_center_distance_px: float
    cross_view_reassociation_max_distance_px: float
    image_width: int
    image_height: int
    aim_service_timeout: float
    observation_input_timeout: float
    observation_search_timeout: float
    observation_max_plans: int
    robot_description: str
    vision_action_name: str
    aim_service_name: str
    spray_action_name: str
    diseased_target_detection_topic: str
    vision_target_topic: str
    selected_target_topic: str
    inference_mode_topic: str
    motion_locked_topic: str
    camera_info_topic: str
    joint_state_topic: str

    @classmethod
    def from_node(cls, node):
        """从 ROS 参数创建经过完整合法性校验的不可变配置。"""
        observation = observation_parameters(node)
        recenter = target_recenter_parameters(node)
        observation_mode, presets = joint_preset_parameters(node)
        max_alignment_attempts = int(
            _value(node, 'max_alignment_attempts'))
        max_targets_per_tree = int(_value(node, 'max_targets_per_tree'))
        if max_alignment_attempts <= 0:
            raise ValueError('max_alignment_attempts must be positive')
        if max_targets_per_tree < 0:
            raise ValueError('max_targets_per_tree must be non-negative')
        side_epsilon = float(_value(node, 'joint_preset_side_epsilon_m'))
        if not math.isfinite(side_epsilon) or side_epsilon < 0.0:
            raise ValueError(
                'joint_preset_side_epsilon_m must be finite and non-negative')
        post_spray_home_delay = float(
            _value(node, 'post_spray_home_delay_sec'))
        if (not math.isfinite(post_spray_home_delay) or
                post_spray_home_delay < 0.0):
            raise ValueError(
                'post_spray_home_delay_sec must be finite and non-negative')
        view_detection_duration = float(
            _value(node, 'view_detection_duration_sec'))
        target_presence_window = float(
            _value(node, 'target_presence_window_sec'))
        target_presence_ratio = float(
            _value(node, 'target_presence_ratio'))
        target_presence_min_frames = int(
            _value(node, 'target_presence_min_frames'))
        if (not math.isfinite(view_detection_duration) or
                view_detection_duration <= 0.0):
            raise ValueError(
                'view_detection_duration_sec must be finite and positive')
        if (not math.isfinite(target_presence_window) or
                not 0.0 < target_presence_window <= view_detection_duration):
            raise ValueError(
                'target_presence_window_sec must be positive and no greater '
                'than view_detection_duration_sec')
        if (not math.isfinite(target_presence_ratio) or
                not 0.0 <= target_presence_ratio <= 1.0):
            raise ValueError(
                'target_presence_ratio must be between zero and one')
        if target_presence_min_frames <= 0:
            raise ValueError('target_presence_min_frames must be positive')
        return cls(
            home=joint_parameter(node, 'home_pose'),
            min_duration=float(_value(node, 'min_spray_duration')),
            max_duration=float(_value(node, 'max_spray_duration')),
            post_spray_home_delay=post_spray_home_delay,
            vision_timeout=float(_value(node, 'vision_timeout_sec')),
            downstream_server_timeout=float(
                _value(node, 'downstream_server_timeout_sec')),
            downstream_margin=float(
                _value(node, 'downstream_result_margin_sec')),
            camera_frame=str(_value(node, 'camera_frame')),
            base_frame=str(_value(node, 'base_frame')),
            spray_on_alignment_failure=bool(
                _value(node, 'spray_on_alignment_failure')),
            observation_mode=observation_mode,
            joint_preset_positions=presets,
            joint_preset_side_epsilon_m=side_epsilon,
            observation=MappingProxyType(observation),
            recenter=MappingProxyType(recenter),
            max_alignment_attempts=max_alignment_attempts,
            max_targets_per_tree=max_targets_per_tree,
            confirmation_frames=int(_value(node, 'confirmation_frames')),
            detection_timeout=float(_value(node, 'detection_timeout_sec')),
            fruit_collection_settle=float(
                _value(node, 'fruit_collection_settle_sec')),
            view_detection_duration=view_detection_duration,
            target_presence_window=target_presence_window,
            target_presence_ratio=target_presence_ratio,
            target_presence_min_frames=target_presence_min_frames,
            processed_iou_threshold=float(
                _value(node, 'processed_iou_threshold')),
            processed_center_distance_px=float(
                _value(node, 'processed_center_distance_px')),
            cross_view_reassociation_max_distance_px=float(
                _value(node, 'cross_view_reassociation_max_distance_px')),
            image_width=int(_value(node, 'image_width')),
            image_height=int(_value(node, 'image_height')),
            aim_service_timeout=float(
                _value(node, 'aim_service_timeout_sec')),
            observation_input_timeout=float(
                _value(node, 'observation_input_timeout_sec')),
            observation_search_timeout=float(
                _value(node, 'observation_search_timeout_sec')),
            observation_max_plans=int(_value(node, 'observation_max_plans')),
            robot_description=str(_value(node, 'robot_description')),
            vision_action_name=str(_value(node, 'vision_action_name')),
            aim_service_name=str(_value(node, 'aim_service_name')),
            spray_action_name=str(_value(node, 'spray_action_name')),
            diseased_target_detection_topic=str(
                _value(node, 'diseased_target_detection_topic')),
            vision_target_topic=str(_value(node, 'vision_target_topic')),
            selected_target_topic=str(_value(node, 'selected_target_topic')),
            inference_mode_topic=str(_value(node, 'inference_mode_topic')),
            motion_locked_topic=str(_value(node, 'motion_locked_topic')),
            camera_info_topic=str(_value(node, 'camera_info_topic')),
            joint_state_topic=str(_value(node, 'joint_state_topic')),
        )
