"""Shared observation-candidate contract for IK and joint-preset strategies."""

from dataclasses import dataclass
import math


@dataclass
class ObservationCandidate:
    """One observation candidate and its geometry, IK and ranking result."""

    candidate_id: str
    distance_m: float
    camera_height_m: float
    azimuth_deg: float
    camera_position: tuple
    camera_quat: tuple
    tool_position: tuple
    tool_quat: tuple
    visible: bool
    visible_margin_px: float
    camera_reach_m: float = 0.0
    visible_fraction: float = 0.0
    projected_bbox: tuple = ()
    target_u_px: float = 0.0
    target_v_px: float = 0.0
    ik_joints: tuple = None
    condition_number: float = math.inf
    min_joint_margin_rad: float = 0.0
    joint_motion_norm: float = math.inf
    nozzle_plane_distance_m: float = math.nan
    nozzle_plane_error_m: float = math.inf
    nozzle_axis_plane_intersection_m: float = math.nan
    rejection_reason: str = ''
    selection_phase: str = 'unranked'
    observation_mode: str = 'ik'
    joint_positions: tuple = ()


def build_candidate(
        candidate_id, distance_m, camera_height_m, azimuth_deg,
        camera_position, camera_quat, tool_position, tool_quat,
        visible, visible_margin_px, visible_fraction=0.0,
        projected_bbox=(), target_u_px=0.0, target_v_px=0.0,
        rejection_reason='', observation_mode='ik', joint_positions=(),
        camera_reach_m=0.0):
    return ObservationCandidate(
        candidate_id=candidate_id,
        distance_m=distance_m,
        camera_height_m=camera_height_m,
        azimuth_deg=float(azimuth_deg),
        camera_position=camera_position,
        camera_quat=camera_quat,
        tool_position=tool_position,
        tool_quat=tool_quat,
        visible=bool(visible),
        visible_margin_px=float(visible_margin_px),
        camera_reach_m=float(camera_reach_m),
        visible_fraction=float(visible_fraction),
        projected_bbox=tuple(float(value) for value in projected_bbox),
        target_u_px=float(target_u_px),
        target_v_px=float(target_v_px),
        rejection_reason=rejection_reason,
        observation_mode=str(observation_mode),
        joint_positions=tuple(float(value) for value in joint_positions),
    )
