"""Pure mission-editor geometry and JSON persistence for the Qt route tool."""

import datetime
import json
import math
import os
from dataclasses import dataclass

from geometry_msgs.msg import Pose


DEFAULT_SAVE_DIRECTORY = os.path.expanduser('~')

# These values are the installed ``alicia_mount_joint`` transform.  The
# Qt editor deliberately uses the same fixed geometry as the real route
# capture tools: a tree click is converted into alicia_base_link coordinates,
# not vehicle-base coordinates.
ARM_BASE_FORWARD_OFFSET_M = -0.40
ARM_BASE_LEFT_OFFSET_M = 0.0
ARM_BASE_YAW_RAD = math.pi
SIDE_EPSILON_M = 0.05
TREE_ROOT_RADIUS_M = 0.16
TREE_CANOPY_RADIUS_M = 0.55
TREE_CANOPY_SEGMENTS = 48

# Gazebo 的树干碰撞圆柱半径约为 0.20 m。仿真静态地图使用 0.25 m
# 的保守树干圆，并沿用 Nav2 的 0.55 m 膨胀半径。该预检只在仿真
# 启用，用来阻止把停车位录在精确规划也无法到达的树干安全区内。
SIM_TREE_TRUNK_RADIUS_M = 0.25
SIM_NAV_FOOTPRINT_HALF_LENGTH_M = 0.725
SIM_NAV_FOOTPRINT_HALF_WIDTH_M = 0.375
SIM_NAV_FOOTPRINT_PADDING_M = 0.05
SIM_NAV_INFLATION_RADIUS_M = 0.55
SIM_NAV_MIN_PARKING_CLEARANCE_M = 0.05

POINT_INSPECT = 'INSPECT'
POINT_TRANSIT = 'TRANSIT'
POINT_TYPES = (POINT_TRANSIT, POINT_INSPECT)

WORK_SIDE_UNSPECIFIED = 'UNSPECIFIED'
WORK_SIDE_LEFT = 'LEFT'
WORK_SIDE_RIGHT = 'RIGHT'
ARM_ANCHOR_POSE_REFERENCE = 'alicia_base_link_xy_vehicle_yaw'
DEFAULT_ARM_SPRAY_DURATION_SEC = 3.0
MIN_ARM_SPRAY_DURATION_SEC = 0.2
MAX_ARM_SPRAY_DURATION_SEC = 10.0


def non_overwriting_json_path(path):
    """Return ``path`` or a numbered sibling without overwriting a mission."""
    root, extension = os.path.splitext(os.path.expanduser(path))
    extension = extension or '.json'
    candidate = root + extension
    suffix = 1
    while os.path.exists(candidate):
        candidate = f'{root}_{suffix:02d}{extension}'
        suffix += 1
    return candidate


def timestamped_mission_path(directory=DEFAULT_SAVE_DIRECTORY, now=None):
    """Create the default save path for one distinct manual mission export."""
    timestamp = (now or datetime.datetime.now()).strftime('%Y%m%d_%H%M%S')
    return non_overwriting_json_path(
        os.path.join(os.path.expanduser(directory),
                     f'navigation_points_{timestamp}.json'))


def route_timeline(points):
    """Return the user-facing relay contract for the submitted route."""
    entries = []
    previous = '起点'
    for index, point in enumerate(points, start=1):
        wide = 'ON' if point.wide_spray_on_approach else 'OFF'
        current = f'{index}:{point.point_type}'
        item = f'{previous} --[广域={wide}]--> {current}'
        if point.point_type == POINT_INSPECT:
            item += f' 病株={point.arm_spray_duration_sec:.1f}s'
        if point.dwell_time_sec > 0.0:
            item += f' 停留={point.dwell_time_sec:.1f}s'
        entries.append(item)
        previous = current
    return ' | '.join(entries)


def arm_anchor_from_vehicle_pose(vehicle_pose):
    """Return the operator-facing arm-base anchor for a vehicle pose."""
    arm_anchor = copy_pose(vehicle_pose)
    yaw = pose_yaw(vehicle_pose)
    cosine, sine = math.cos(yaw), math.sin(yaw)
    arm_anchor.position.x += (
        cosine * ARM_BASE_FORWARD_OFFSET_M
        - sine * ARM_BASE_LEFT_OFFSET_M)
    arm_anchor.position.y += (
        sine * ARM_BASE_FORWARD_OFFSET_M
        + cosine * ARM_BASE_LEFT_OFFSET_M)
    return arm_anchor


def vehicle_pose_from_arm_anchor(arm_anchor_pose):
    """Convert an operator-facing arm-base click into a Nav2 vehicle goal."""
    vehicle_pose = copy_pose(arm_anchor_pose)
    yaw = pose_yaw(arm_anchor_pose)
    cosine, sine = math.cos(yaw), math.sin(yaw)
    vehicle_pose.position.x -= (
        cosine * ARM_BASE_FORWARD_OFFSET_M
        - sine * ARM_BASE_LEFT_OFFSET_M)
    vehicle_pose.position.y -= (
        sine * ARM_BASE_FORWARD_OFFSET_M
        + cosine * ARM_BASE_LEFT_OFFSET_M)
    return vehicle_pose


def tree_offset_from_arm_anchor(arm_anchor_pose, tree_pose):
    """Return signed Alicia-frame XY for a tree clicked in ``map``."""
    arm_yaw = pose_yaw(arm_anchor_pose) + ARM_BASE_YAW_RAD
    arm_cosine, arm_sine = math.cos(arm_yaw), math.sin(arm_yaw)
    dx = tree_pose.position.x - arm_anchor_pose.position.x
    dy = tree_pose.position.y - arm_anchor_pose.position.y
    return (
        arm_cosine * dx + arm_sine * dy,
        -arm_sine * dx + arm_cosine * dy,
    )


def tree_offset_from_docking(docking_pose, tree_pose):
    """Compatibility helper for legacy vehicle-base docking poses."""
    return tree_offset_from_arm_anchor(
        arm_anchor_from_vehicle_pose(docking_pose), tree_pose)


def work_side_from_tree_y(tree_y_m):
    tree_y_m = float(tree_y_m)
    if not math.isfinite(tree_y_m) or abs(tree_y_m) < SIDE_EPSILON_M:
        return WORK_SIDE_UNSPECIFIED
    return WORK_SIDE_LEFT if tree_y_m > 0.0 else WORK_SIDE_RIGHT


def valid_work_side(point):
    """Return ``None`` when an inspect point's declared side is consistent."""
    if point.point_type != POINT_INSPECT:
        return None
    inferred = work_side_from_tree_y(point.tree_y_m)
    if inferred == WORK_SIDE_UNSPECIFIED:
        return '病株相对机械臂基座的 Y 不能接近 0'
    if point.work_side != inferred:
        return (
            f'病株侧位与相对 Y 不一致: Y={point.tree_y_m:.3f} m, '
            f'应为 {inferred}')
    return None


def simulation_parking_clearance_m(arm_anchor_pose, tree_pose):
    """Return clearance from the padded vehicle footprint to one tree's map cost."""
    vehicle_pose = vehicle_pose_from_arm_anchor(arm_anchor_pose)
    yaw = pose_yaw(vehicle_pose)
    dx = tree_pose.position.x - vehicle_pose.position.x
    dy = tree_pose.position.y - vehicle_pose.position.y
    cosine, sine = math.cos(yaw), math.sin(yaw)
    local_x = cosine * dx + sine * dy
    local_y = -sine * dx + cosine * dy
    half_length = (SIM_NAV_FOOTPRINT_HALF_LENGTH_M +
                   SIM_NAV_FOOTPRINT_PADDING_M)
    half_width = (SIM_NAV_FOOTPRINT_HALF_WIDTH_M +
                  SIM_NAV_FOOTPRINT_PADDING_M)
    nearest_x = min(max(local_x, -half_length), half_length)
    nearest_y = min(max(local_y, -half_width), half_width)
    footprint_distance = math.hypot(local_x - nearest_x, local_y - nearest_y)
    return (footprint_distance - SIM_TREE_TRUNK_RADIUS_M -
            SIM_NAV_INFLATION_RADIUS_M)


def simulation_parking_clearance_error(point, enabled=False):
    """Return a simulation-only recording error for an unsafe inspect parking pose."""
    if (not enabled or point.point_type != POINT_INSPECT or
            point.tree_pose is None):
        return None
    clearance = simulation_parking_clearance_m(point.pose, point.tree_pose)
    if clearance >= SIM_NAV_MIN_PARKING_CLEARANCE_M:
        return None
    return (
        '仿真停车位过近：车辆 footprint 至树干/膨胀区净余量 '
        f'{clearance:.2f} m，小于 {SIM_NAV_MIN_PARKING_CLEARANCE_M:.2f} m；'
        '请将机械臂基座停靠位远离树中心后重新记录')


def copy_pose(source):
    pose = Pose()
    pose.position.x = source.position.x
    pose.position.y = source.position.y
    pose.position.z = source.position.z
    pose.orientation.x = source.orientation.x
    pose.orientation.y = source.orientation.y
    pose.orientation.z = source.orientation.z
    pose.orientation.w = source.orientation.w
    return pose


def pose_yaw(pose):
    return math.atan2(
        2.0 * (pose.orientation.w * pose.orientation.z
               + pose.orientation.x * pose.orientation.y),
        1.0 - 2.0 * (pose.orientation.y ** 2 + pose.orientation.z ** 2),
    )


def pose_to_json(pose):
    return {
        'position': {
            'x': pose.position.x,
            'y': pose.position.y,
            'z': pose.position.z,
        },
        'orientation': {
            'x': pose.orientation.x,
            'y': pose.orientation.y,
            'z': pose.orientation.z,
            'w': pose.orientation.w,
        },
    }


def pose_from_json(data):
    pose = Pose()
    pose.position.x = float(data['position']['x'])
    pose.position.y = float(data['position']['y'])
    pose.position.z = float(data['position'].get('z', 0.0))
    pose.orientation.x = float(data['orientation'].get('x', 0.0))
    pose.orientation.y = float(data['orientation'].get('y', 0.0))
    pose.orientation.z = float(data['orientation']['z'])
    pose.orientation.w = float(data['orientation']['w'])
    return pose


@dataclass
class WorkPoint:
    pose: Pose
    tree_x_m: float = 0.0
    tree_y_m: float = 0.0
    tree_base_z_m: float = 0.0
    point_type: str = POINT_TRANSIT
    work_side: str = WORK_SIDE_UNSPECIFIED
    wide_spray_on_approach: bool = False
    arm_spray_duration_sec: float = DEFAULT_ARM_SPRAY_DURATION_SEC
    dwell_time_sec: float = 0.0
    tree_pose: Pose | None = None


class MissionEditor:
    """Qt 保存的人工任务；所有停靠点均由操作员在 RViz 中记录。"""

    schema_version = 8

    def __init__(self, spray_duration=DEFAULT_ARM_SPRAY_DURATION_SEC):
        spray_duration = float(spray_duration)
        if not (MIN_ARM_SPRAY_DURATION_SEC <= spray_duration <=
                MAX_ARM_SPRAY_DURATION_SEC):
            raise ValueError(
                'default_arm_spray_duration_sec must be within '
                f'{MIN_ARM_SPRAY_DURATION_SEC:.1f}–'
                f'{MAX_ARM_SPRAY_DURATION_SEC:.1f}')
        self.start_pose = None
        self.points = []
        self.spray_duration = spray_duration
        self.return_home_after_mission = False

    def add_point(self, pose, tree_x_m=0.0, tree_y_m=0.0, tree_base_z_m=0.0,
                  point_type=POINT_TRANSIT,
                  work_side=WORK_SIDE_UNSPECIFIED,
                  wide_spray_on_approach=False,
                  arm_spray_duration_sec=None,
                  dwell_time_sec=0.0,
                  tree_pose=None):
        if point_type not in POINT_TYPES:
            raise ValueError(f'unsupported point type: {point_type}')
        if point_type == POINT_INSPECT and work_side == WORK_SIDE_UNSPECIFIED:
            work_side = work_side_from_tree_y(tree_y_m)
        if arm_spray_duration_sec is None:
            arm_spray_duration_sec = self.spray_duration
        self.points.append(WorkPoint(
            copy_pose(pose), float(tree_x_m), float(tree_y_m),
            float(tree_base_z_m), point_type, work_side,
            bool(wide_spray_on_approach), float(arm_spray_duration_sec),
            float(dwell_time_sec),
            copy_pose(tree_pose) if tree_pose is not None else None))

    def save(self, path):
        data = {
            'schema_version': self.schema_version,
            'pose_reference': ARM_ANCHOR_POSE_REFERENCE,
            'start_pose': (
                pose_to_json(self.start_pose) if self.start_pose else None),
            'spray_duration': self.spray_duration,
            'return_home_after_mission': self.return_home_after_mission,
            'route_points': [
                {'pose': pose_to_json(point.pose),
                 'point_type': point.point_type,
                 'tree_x_m': point.tree_x_m,
                 'tree_y_m': point.tree_y_m,
                 'tree_base_z_m': point.tree_base_z_m,
                 'tree_pose': (
                     pose_to_json(point.tree_pose)
                     if point.tree_pose is not None else None),
                 'work_side': point.work_side,
                 'wide_spray_on_approach': point.wide_spray_on_approach,
                 'arm_spray_duration_sec': point.arm_spray_duration_sec,
                 'dwell_time_sec': point.dwell_time_sec}
                for point in self.points
            ],
        }
        with open(path, 'w', encoding='utf-8') as stream:
            json.dump(data, stream, indent=2, ensure_ascii=False)

    def load(self, path):
        with open(path, encoding='utf-8') as stream:
            data = json.load(stream)
        version = data.get('schema_version')
        if version != self.schema_version:
            raise ValueError(
                f'unsupported navigation file schema_version: {version!r}')
        if data.get('pose_reference') != ARM_ANCHOR_POSE_REFERENCE:
            raise ValueError(
                'schema v8 requires pose_reference='
                f'{ARM_ANCHOR_POSE_REFERENCE!r}')

        self.start_pose = (
            pose_from_json(data['start_pose'])
            if data.get('start_pose') else None)
        self.spray_duration = float(data.get('spray_duration', 3.0))
        self.return_home_after_mission = bool(
            data.get('return_home_after_mission', False))
        self.points = []
        for item in data.get('route_points', []):
            point_type = str(item.get('point_type', POINT_TRANSIT))
            if point_type not in POINT_TYPES:
                raise ValueError(f'unsupported point type: {point_type}')
            tree_pose_data = item.get('tree_pose')
            self.points.append(WorkPoint(
                pose_from_json(item['pose']),
                float(item.get('tree_x_m', 0.0)),
                float(item.get('tree_y_m', 0.0)),
                float(item.get('tree_base_z_m', 0.0)),
                point_type,
                str(item.get('work_side', WORK_SIDE_UNSPECIFIED)),
                bool(item.get('wide_spray_on_approach', False)),
                float(item.get('arm_spray_duration_sec',
                               data.get('spray_duration', 3.0))),
                float(item.get('dwell_time_sec', 0.0)),
                pose_from_json(tree_pose_data) if tree_pose_data else None,
            ))
