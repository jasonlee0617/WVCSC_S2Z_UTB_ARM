# mission_request.py
# ============================================================================
# Qt/RViz 人工任务请求校验与坐标转换（纯逻辑，不依赖 ROS2）
# ============================================================================

import math

from .core import PointType, Target, WorkSide, tree_hint_from_arm_base_offset


def validate_manual_request(
        request, *, map_frame, max_targets, max_abs_coordinate,
        min_spray_duration, max_spray_duration, arm_base_forward_offset,
        arm_base_left_offset, arm_base_yaw):
    """Return validated Targets and HOME pose from one LoadManualMission request."""
    if request.header.frame_id != map_frame:
        raise ValueError(f'frame must be {map_frame}')
    if not request.mission_id.strip() or not request.targets:
        raise ValueError('mission_id and targets are required')
    if len(request.targets) > int(max_targets):
        raise ValueError(f'target count exceeds limit {max_targets}')

    bound = float(max_abs_coordinate)
    home_pose = pose_to_xy_yaw(request.home_pose, 'home_pose')
    if abs(home_pose[0]) > bound or abs(home_pose[1]) > bound:
        raise ValueError('home_pose is out of bounds')

    seen = set()
    targets = []
    for item in request.targets:
        target_id = item.target_id.strip()
        if not target_id or target_id in seen:
            raise ValueError('target_id must be non-empty and unique')
        point_type = int(getattr(item, 'point_type', PointType.INSPECT))
        if point_type not in set(PointType):
            raise ValueError(f'{target_id}: unsupported point_type')
        work_side = int(getattr(item, 'work_side', WorkSide.UNSPECIFIED))
        if work_side not in set(WorkSide):
            raise ValueError(f'{target_id}: unsupported work_side')
        docking = pose_to_xy_yaw(item.docking_pose, f'{target_id}.docking_pose')
        if abs(docking[0]) > bound or abs(docking[1]) > bound:
            raise ValueError(f'{target_id}: docking pose out of bounds')
        dwell_time = float(getattr(item, 'dwell_time_sec', 0.0))
        if not math.isfinite(dwell_time) or dwell_time < 0.0:
            raise ValueError(f'{target_id}: dwell_time_sec must be non-negative')
        wide_spray_on_approach = bool(getattr(
            item, 'wide_spray_on_approach', False))
        if point_type != PointType.INSPECT:
            targets.append(Target(
                target_id, 0.0, 0.0, 0.0, 0.0, docking,
                point_type=point_type,
                wide_spray_on_approach=wide_spray_on_approach,
                dwell_time_sec=dwell_time,
                work_side=work_side))
            seen.add(target_id)
            continue

        configured_duration = float(getattr(item, 'arm_spray_duration_sec', 0.0))
        duration = configured_duration if configured_duration > 0.0 else float(
            item.spray_duration)
        if (not math.isfinite(duration) or
                not min_spray_duration <= duration <= max_spray_duration):
            raise ValueError(f'{target_id}: spray_duration out of range')
        tree_x_m = float(item.tree_x_m)
        tree_y_m = float(item.tree_y_m)
        tree_base_z = float(item.tree_base_z_m)
        if not all(math.isfinite(value) for value in (
                tree_x_m, tree_y_m, tree_base_z)):
            raise ValueError(f'{target_id}: non-finite arm-base tree offset')
        if math.hypot(tree_x_m, tree_y_m) < 1e-6:
            raise ValueError(f'{target_id}: arm-base tree offset is zero')
        if work_side != WorkSide.UNSPECIFIED:
            expected_side = WorkSide.LEFT if tree_y_m > 0.0 else WorkSide.RIGHT
            if abs(tree_y_m) <= 0.05 or work_side != expected_side:
                raise ValueError(
                    f'{target_id}: work_side conflicts with signed tree Y')
        tree_hint = tree_hint_from_arm_base_offset(
            docking, tree_x_m, tree_y_m, tree_base_z,
            arm_base_forward_offset, arm_base_left_offset, arm_base_yaw)
        if abs(tree_hint[0]) > bound or abs(tree_hint[1]) > bound:
            raise ValueError(f'{target_id}: derived tree hint out of bounds')
        targets.append(Target(
            target_id, tree_hint[0], tree_hint[1], tree_hint[2], duration,
            docking, tree_x_m, tree_y_m, point_type, wide_spray_on_approach,
            dwell_time, work_side))
        seen.add(target_id)
    return targets, home_pose


def pose_to_xy_yaw(pose, label):
    values = (
        pose.position.x, pose.position.y, pose.position.z,
        pose.orientation.x, pose.orientation.y,
        pose.orientation.z, pose.orientation.w)
    if not all(math.isfinite(value) for value in values):
        raise ValueError(f'{label}: non-finite pose')
    norm = math.sqrt(sum(value * value for value in values[3:]))
    if norm < 1e-6:
        raise ValueError(f'{label}: invalid orientation')
    x, y, z, w = (value / norm for value in values[3:])
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return pose.position.x, pose.position.y, yaw
