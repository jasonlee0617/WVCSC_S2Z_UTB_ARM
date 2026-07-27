#!/usr/bin/env python3
"""Manual single-point and multi-point mission editor for WVCSC."""

import datetime
import json
import math
import os
import sys
import uuid
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Point, Pose, PoseStamped, PoseWithCovarianceStamped
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QColor
from PyQt5.QtWidgets import (
    QApplication,
    QAbstractItemView,
    QCheckBox,
    QComboBox,
    QFileDialog,
    QGridLayout,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QMessageBox,
    QPushButton,
    QTableWidget,
    QTableWidgetItem,
    QTextEdit,
    QSplitter,
    QVBoxLayout,
    QWidget,
)
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from std_srvs.srv import Empty, Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray
from wvcsc_interfaces.msg import (
    ManualMissionTarget,
    MissionStatus,
)
from wvcsc_interfaces.srv import LoadManualMission
from wvcsc_bringup.qt_image_viewer import RosImagePanel


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
POINT_FINISH = 'FINISH'
POINT_TYPES = (POINT_TRANSIT, POINT_INSPECT, POINT_FINISH)

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


def remove_opencv_qt_plugin_override(environment=None):
    """Prevent a pip OpenCV wheel from selecting its incompatible Qt plugin.

    ``opencv-python`` wheels may export ``QT_QPA_PLATFORM_PLUGIN_PATH`` as
    ``.../site-packages/cv2/qt/plugins``.  This UI is PyQt5-based, so that
    plugin must never be selected for its QApplication.
    """
    environment = os.environ if environment is None else environment
    for name in ('QT_QPA_PLATFORM_PLUGIN_PATH', 'QT_PLUGIN_PATH'):
        value = environment.get(name, '')
        normalized = value.replace('\\', '/')
        if '/cv2/qt/plugins' in normalized:
            environment.pop(name, None)
    font_path = environment.get('QT_QPA_FONTDIR', '')
    if '/cv2/qt/' in font_path.replace('\\', '/'):
        environment.pop('QT_QPA_FONTDIR', None)


def arm_anchor_from_vehicle_pose(vehicle_pose):
    """Return the operator-facing arm-base anchor for a vehicle pose.

    The XY point is ``alicia_base_link``.  Its orientation deliberately keeps
    the vehicle heading because the RViz 2D Goal arrow means "vehicle front",
    not the Alicia arm's local +X direction.
    """
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


def ik_recording_range_error(point, observation_mode, minimum_m=0.85,
                             maximum_m=1.45):
    """Return a recording-time IK safety error for an inspection point.

    RViz records the arm-base anchor and the tree centre independently.  The
    arm keeps its wider runtime interlock (0.80--1.50 m), while this editor
    leaves a 5 cm margin on both sides so an operator cannot submit a task
    which is already on the hard limit before Nav2 docking variation.
    """
    if (str(observation_mode).strip().lower() != 'ik' or
            point.point_type != POINT_INSPECT):
        return None
    distance = math.hypot(float(point.tree_x_m), float(point.tree_y_m))
    if not math.isfinite(distance):
        return 'IK模式病株相对机械臂基座距离无效'
    minimum_m = float(minimum_m)
    maximum_m = float(maximum_m)
    if minimum_m <= distance <= maximum_m:
        return None
    return (
        f'IK模式病株距离 {distance:.2f} m 超出录入范围 '
        f'{minimum_m:.2f}–{maximum_m:.2f} m；'
        '请重新设置机械臂基座停靠位与树中心')


def simulation_parking_clearance_m(arm_anchor_pose, tree_pose):
    """Return clearance from the padded vehicle footprint to one tree's map cost.

    RViz records ``alicia_base_link`` ground projections, whereas Nav2 moves
    ``base_footprint``.  The calculation therefore first converts the anchor
    to the actual vehicle pose, rotates the tree into the vehicle frame, and
    measures the shortest distance to its padded rectangle.  A positive
    result is free space remaining after the static trunk and inflation cost.
    """
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

    schema_version = 7

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
        self.return_home_after_finish = False
        self.load_warning = None

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
        if point_type == POINT_FINISH:
            wide_spray_on_approach = False
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
            'return_home_after_finish': self.return_home_after_finish,
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
        if version not in (3, 4, 5, 6, self.schema_version):
            raise ValueError(
                f'unsupported navigation file schema_version: {version!r}')
        if (version in (5, 6, self.schema_version) and
                data.get('pose_reference') != ARM_ANCHOR_POSE_REFERENCE):
            raise ValueError(
                'schema v5/v6/v7 requires pose_reference='
                f'{ARM_ANCHOR_POSE_REFERENCE!r}')
        legacy_vehicle_pose = version in (3, 4)

        def load_pose(data_item):
            pose = pose_from_json(data_item)
            return (arm_anchor_from_vehicle_pose(pose)
                    if legacy_vehicle_pose else pose)

        self.start_pose = (
            load_pose(data['start_pose'])
            if data.get('start_pose') else None)
        self.spray_duration = float(data.get('spray_duration', 3.0))
        self.return_home_after_finish = bool(
            data.get('return_home_after_finish', False))
        self.load_warning = None
        if version == 3:
            # Schema v3/v4 stored vehicle-base Nav2 goals.  Convert them to
            # the new operator-facing arm-base anchor without changing the
            # eventual vehicle path submitted to Nav2.
            raw_points = data.get('targets', [])
            self.points = [
                WorkPoint(
                    load_pose(item['pose']),
                    float(item['tree_x_m']),
                    float(item['tree_y_m']),
                    float(item.get('tree_base_z_m', 0.0)),
                    POINT_INSPECT,
                    work_side_from_tree_y(item['tree_y_m']),
                    False,
                    float(data.get('spray_duration', 3.0)),
                    0.0,
                    None,
                )
                for item in raw_points
            ]
            self.load_warning = (
                '已导入旧版车辆基座任务：已转换为机械臂基座点击语义。'
                '所有点均按病株检查点处理；请复核点类型、广域喷洒和侧位。')
            return

        if version in (5, 6) and 'auto_start' in data:
            self.load_warning = (
                '已导入旧版任务：已忽略 auto_start。加载只预览任务，'
                '请人工点击“开始任务”。')

        self.points = []
        for item in data.get('route_points', []):
            point_type = str(item.get('point_type', POINT_TRANSIT))
            if point_type not in POINT_TYPES:
                raise ValueError(f'unsupported point type: {point_type}')
            tree_pose_data = item.get('tree_pose')
            self.points.append(WorkPoint(
                load_pose(item['pose']),
                float(item.get('tree_x_m', 0.0)),
                float(item.get('tree_y_m', 0.0)),
                float(item.get('tree_base_z_m', 0.0)),
                point_type,
                str(item.get('work_side', WORK_SIDE_UNSPECIFIED)),
                (False if point_type == POINT_FINISH else
                 bool(item.get('wide_spray_on_approach', False))),
                float(item.get('arm_spray_duration_sec',
                               data.get('spray_duration', 3.0))),
                float(item.get('dwell_time_sec', 0.0)),
                pose_from_json(tree_pose_data) if tree_pose_data else None,
            ))


class Nav2QtNode(Node):
    def __init__(self):
        super().__init__('nav2_qt')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('initial_pose_topic', '/initialpose')
        self.declare_parameter('goal_pose_topic', '/manual_goal_pose')
        self.declare_parameter('marker_topic', '/waypoints')
        self.declare_parameter('require_global_relocalization_service', True)
        self.declare_parameter('show_sim_spray_status', False)
        self.declare_parameter('simulation_parking_clearance_check', False)
        self.declare_parameter(
            'default_arm_spray_duration_sec',
            DEFAULT_ARM_SPRAY_DURATION_SEC)
        # IK mode has a stricter recording envelope than the arm's runtime
        # interlock.  Joint presets deliberately bypass this radial check.
        self.declare_parameter('observation_mode', 'joint_presets')
        self.declare_parameter('ik_recording_range_min_m', 0.85)
        self.declare_parameter('ik_recording_range_max_m', 1.45)
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.require_global_relocalization_service = bool(
            self.get_parameter('require_global_relocalization_service').value)
        self.show_sim_spray_status = bool(
            self.get_parameter('show_sim_spray_status').value)
        self.simulation_parking_clearance_check = bool(
            self.get_parameter('simulation_parking_clearance_check').value)
        self.default_arm_spray_duration_sec = float(
            self.get_parameter('default_arm_spray_duration_sec').value)
        self.observation_mode = str(
            self.get_parameter('observation_mode').value).strip().lower()
        self.ik_recording_range_min_m = float(
            self.get_parameter('ik_recording_range_min_m').value)
        self.ik_recording_range_max_m = float(
            self.get_parameter('ik_recording_range_max_m').value)
        if self.observation_mode not in {'ik', 'joint_presets'}:
            raise ValueError('observation_mode must be ik or joint_presets')
        if not (0.0 < self.ik_recording_range_min_m <
                self.ik_recording_range_max_m):
            raise ValueError('invalid IK recording range')
        if not (MIN_ARM_SPRAY_DURATION_SEC <=
                self.default_arm_spray_duration_sec <=
                MAX_ARM_SPRAY_DURATION_SEC):
            raise ValueError(
                'default_arm_spray_duration_sec must be within '
                f'{MIN_ARM_SPRAY_DURATION_SEC:.1f}–'
                f'{MAX_ARM_SPRAY_DURATION_SEC:.1f}')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.latest_initial_pose = None
        self.initial_pose_sequence = 0
        self.latest_goal_pose = None
        self.goal_sequence = 0
        self.status = None
        self.sim_relay_active = {1: False, 2: False}

        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            str(self.get_parameter('initial_pose_topic').value),
            self._on_initial_pose, 10)
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter('goal_pose_topic').value),
            self._on_goal_pose, 10)
        self.create_subscription(MissionStatus, '/mission/status',
                                 self._on_status, latched)
        if self.show_sim_spray_status:
            self.create_subscription(
                Bool, '/relay/sim/channel_1_active',
                lambda message: self._on_sim_relay(1, message), latched)
            self.create_subscription(
                Bool, '/relay/sim/channel_2_active',
                lambda message: self._on_sim_relay(2, message), latched)
        self.marker_pub = self.create_publisher(
            MarkerArray, str(self.get_parameter('marker_topic').value), 10)
        self.service_clients = {
            'load': self.create_client(LoadManualMission,
                                       '/mission/load_manual'),
            'start': self.create_client(Trigger, '/mission/start'),
            'abort_and_home': self.create_client(
                Trigger, '/mission/abort_and_home'),
            'reinitialize_global_localization': self.create_client(
                Empty, '/reinitialize_global_localization'),
            'return_home': self.create_client(Trigger, '/mission/return_home'),
            'reset': self.create_client(Trigger, '/mission/reset'),
        }

    def _on_initial_pose(self, message):
        pose = message.pose.pose
        values = (
            pose.position.x, pose.position.y, pose.position.z,
            pose.orientation.x, pose.orientation.y,
            pose.orientation.z, pose.orientation.w)
        if (message.header.frame_id == self.map_frame and
                all(math.isfinite(value) for value in values)):
            self.latest_initial_pose = copy_pose(pose)
            self.initial_pose_sequence += 1

    def _on_goal_pose(self, message):
        if message.header.frame_id != self.map_frame:
            self.get_logger().warning(
                f'ignored goal in frame {message.header.frame_id}; '
                f'expected {self.map_frame}')
            return
        self.latest_goal_pose = copy_pose(message.pose)
        self.goal_sequence += 1

    def _on_status(self, message):
        self.status = message

    def _on_sim_relay(self, channel, message):
        self.sim_relay_active[channel] = bool(message.data)

    def current_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time())
        except TransformException as error:
            self.get_logger().warning(f'cannot read vehicle pose: {error}')
            return None
        pose = Pose()
        pose.position.x = transform.transform.translation.x
        pose.position.y = transform.transform.translation.y
        pose.position.z = transform.transform.translation.z
        pose.orientation = transform.transform.rotation
        return pose

    @staticmethod
    def _target_constant(name, fallback):
        return int(getattr(ManualMissionTarget, name, fallback))

    def build_manual_request(self, start_pose, points,
                             return_home_after_finish, prefix):
        request = LoadManualMission.Request()
        request.header.stamp = self.get_clock().now().to_msg()
        request.header.frame_id = self.map_frame
        request.mission_id = f'manual_{prefix}_{uuid.uuid4().hex[:8]}'
        request.home_pose = vehicle_pose_from_arm_anchor(start_pose)
        request.return_home_after_finish = return_home_after_finish
        for index, point in enumerate(points, start=1):
            error = valid_work_side(point)
            if error is None:
                error = ik_recording_range_error(
                    point, getattr(self, 'observation_mode', 'joint_presets'),
                    getattr(self, 'ik_recording_range_min_m', 0.85),
                    getattr(self, 'ik_recording_range_max_m', 1.45))
            if error is None:
                error = simulation_parking_clearance_error(
                    point,
                    getattr(self, 'simulation_parking_clearance_check', False))
            if error is not None:
                raise ValueError(f'第 {index} 个点无效：{error}')
            target = ManualMissionTarget()
            target.target_id = f'{prefix}_{index:02d}'
            target.docking_pose = vehicle_pose_from_arm_anchor(point.pose)
            is_inspect = point.point_type == POINT_INSPECT
            target.tree_x_m = point.tree_x_m if is_inspect else 0.0
            target.tree_y_m = point.tree_y_m if is_inspect else 0.0
            target.tree_base_z_m = point.tree_base_z_m if is_inspect else 0.0
            target.spray_duration = (
                float(point.arm_spray_duration_sec)
                if is_inspect else 0.0)
            # These fields are part of the typed real-route extension.  The
            # guard keeps a source checkout usable with an older generated
            # interface until the whole workspace is rebuilt.
            point_type = {
                POINT_INSPECT: self._target_constant('POINT_INSPECT', 0),
                POINT_TRANSIT: self._target_constant('POINT_TRANSIT', 1),
                POINT_FINISH: self._target_constant('POINT_FINISH', 2),
            }[point.point_type]
            work_side = {
                WORK_SIDE_UNSPECIFIED: self._target_constant(
                    'WORK_SIDE_UNSPECIFIED', 0),
                WORK_SIDE_LEFT: self._target_constant('WORK_SIDE_LEFT', 1),
                WORK_SIDE_RIGHT: self._target_constant('WORK_SIDE_RIGHT', 2),
            }[point.work_side]
            for name, value in (
                    ('point_type', point_type),
                    ('wide_spray_on_approach',
                     bool(point.wide_spray_on_approach)
                     if point.point_type != POINT_FINISH else False),
                    ('dwell_time_sec', float(point.dwell_time_sec)),
                    ('work_side', work_side),
                    ('arm_spray_duration_sec',
                     float(point.arm_spray_duration_sec) if is_inspect else 0.0)):
                if hasattr(target, name):
                    setattr(target, name, value)
            request.targets.append(target)
        return request

    def publish_markers(self, editor, candidate, pending_dock=None):
        markers = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        if editor.start_pose is not None:
            markers.markers.append(self._marker(
                editor.start_pose, 'manual_start', 0, 0.2, 0.9, 0.2))
            markers.markers.append(self._vehicle_marker(
                editor.start_pose, 'manual_start_vehicle', 0, 0.2, 0.9, 0.2))
            markers.markers.append(self._mount_line(
                editor.start_pose, 'manual_start_mount', 0, 0.2, 0.9, 0.2))
        for index, point in enumerate(editor.points, start=1):
            color = {
                POINT_TRANSIT: (0.1, 0.6, 1.0),
                POINT_INSPECT: (1.0, 0.75, 0.0),
                POINT_FINISH: (0.15, 0.85, 0.3),
            }[point.point_type]
            markers.markers.append(self._marker(
                point.pose, 'manual_target', index, *color))
            markers.markers.append(self._vehicle_marker(
                point.pose, 'manual_target_vehicle', index, *color))
            markers.markers.append(self._mount_line(
                point.pose, 'manual_target_mount', index, *color))
            markers.markers.append(self._label(point.pose, index, point))
            if point.tree_pose is not None:
                markers.markers.append(
                    self._tree_root_marker(point.tree_pose, index))
                markers.markers.append(
                    self._tree_canopy_marker(point.tree_pose, index))
                markers.markers.append(
                    self._tree_line(point.pose, point.tree_pose, index))
                markers.markers.append(
                    self._tree_distance_label(point.pose, point.tree_pose, index))
                markers.markers.append(self._tree_label(point.tree_pose, index))
        # Cyan line: operator-recorded route after every arm-base click has
        # been converted into the actual vehicle-base Nav2 goal.  It is not a
        # planner prediction; the magenta Path in RViz is recorded odometry.
        route_anchors = []
        if editor.start_pose is not None:
            route_anchors.append(editor.start_pose)
        route_anchors.extend(point.pose for point in editor.points)
        if len(route_anchors) >= 2:
            markers.markers.append(self._vehicle_route_marker(route_anchors))
        if candidate is not None:
            markers.markers.append(self._marker(
                candidate, 'manual_candidate', 1000, 1.0, 0.8, 0.0))
            markers.markers.append(self._vehicle_marker(
                candidate, 'manual_candidate_vehicle', 1000, 1.0, 0.8, 0.0))
            markers.markers.append(self._mount_line(
                candidate, 'manual_candidate_mount', 1000, 1.0, 0.8, 0.0))
        if pending_dock is not None:
            markers.markers.append(self._marker(
                pending_dock, 'manual_pending_inspect_dock', 1001,
                0.75, 0.2, 0.9))
            markers.markers.append(self._vehicle_marker(
                pending_dock, 'manual_pending_inspect_vehicle', 1001,
                0.75, 0.2, 0.9))
        self.marker_pub.publish(markers)

    def _marker(self, pose, namespace, marker_id, red, green, blue):
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose = copy_pose(pose)
        marker.scale.x = 0.55
        marker.scale.y = 0.14
        marker.scale.z = 0.14
        marker.color.r = red
        marker.color.g = green
        marker.color.b = blue
        marker.color.a = 0.9
        return marker

    def _vehicle_marker(self, arm_anchor, namespace, marker_id,
                        red, green, blue):
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = vehicle_pose_from_arm_anchor(arm_anchor)
        marker.scale.x = marker.scale.y = marker.scale.z = 0.16
        marker.color.r = red
        marker.color.g = green
        marker.color.b = blue
        marker.color.a = 0.55
        return marker

    def _mount_line(self, arm_anchor, namespace, marker_id,
                    red, green, blue):
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.025
        marker.color.r = red
        marker.color.g = green
        marker.color.b = blue
        marker.color.a = 0.65
        marker.points.extend([
            arm_anchor.position,
            vehicle_pose_from_arm_anchor(arm_anchor).position,
        ])
        return marker

    def _vehicle_route_marker(self, arm_anchors):
        """Show the recorded vehicle-base route derived from arm-base clicks."""
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'manual_vehicle_route'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.045
        marker.color.r = 0.0
        marker.color.g = 0.95
        marker.color.b = 0.95
        marker.color.a = 0.90
        for arm_anchor in arm_anchors:
            vehicle_pose = vehicle_pose_from_arm_anchor(arm_anchor)
            marker.points.append(Point(
                x=vehicle_pose.position.x,
                y=vehicle_pose.position.y,
                z=0.06,
            ))
        return marker

    def _label(self, pose, index, point):
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'manual_target_label'
        marker.id = index
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose = copy_pose(pose)
        marker.pose.position.z += 0.35
        marker.scale.z = 0.25
        marker.color.r = marker.color.g = marker.color.b = marker.color.a = 1.0
        marker.text = f'{index}: {point.point_type}'
        if point.point_type == POINT_INSPECT:
            marker.text += f' {point.work_side}'
        return marker

    def _tree_root_marker(self, pose, marker_id):
        """Draw the physical tree-trunk footprint at the manually clicked root."""
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'manual_tree_root'
        marker.id = marker_id
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose = copy_pose(pose)
        marker.pose.position.z = 0.015
        marker.scale.x = marker.scale.y = TREE_ROOT_RADIUS_M * 2.0
        marker.scale.z = 0.03
        marker.color.r = 0.45
        marker.color.g = 0.20
        marker.color.b = 0.05
        marker.color.a = 0.95
        return marker

    def _tree_canopy_marker(self, pose, marker_id):
        """Draw the horizontal outer envelope of the current tree model."""
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'manual_tree_canopy'
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.025
        marker.color.r = 1.0
        marker.color.g = 0.72
        marker.color.b = 0.05
        marker.color.a = 0.95
        for step in range(TREE_CANOPY_SEGMENTS + 1):
            angle = 2.0 * math.pi * step / TREE_CANOPY_SEGMENTS
            marker.points.append(Point(
                x=pose.position.x + TREE_CANOPY_RADIUS_M * math.cos(angle),
                y=pose.position.y + TREE_CANOPY_RADIUS_M * math.sin(angle),
                z=0.04,
            ))
        return marker

    def _tree_distance_label(self, arm_anchor, tree, marker_id):
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'manual_tree_distance'
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = (arm_anchor.position.x + tree.position.x) / 2.0
        marker.pose.position.y = (arm_anchor.position.y + tree.position.y) / 2.0
        marker.pose.position.z = 0.24
        marker.scale.z = 0.18
        marker.color.r = marker.color.g = marker.color.b = marker.color.a = 1.0
        distance = math.hypot(
            tree.position.x - arm_anchor.position.x,
            tree.position.y - arm_anchor.position.y)
        marker.text = f'ARM-ROOT: {distance:.2f} m'
        return marker

    def _tree_label(self, pose, marker_id):
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'manual_tree_label'
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose = copy_pose(pose)
        marker.pose.position.z = 0.30
        marker.scale.z = 0.16
        marker.color.r = 1.0
        marker.color.g = 0.85
        marker.color.b = 0.2
        marker.color.a = 1.0
        marker.text = f'ROOT\nCANOPY r={TREE_CANOPY_RADIUS_M:.2f}m'
        return marker

    def _tree_line(self, docking, tree, marker_id):
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'manual_tree_link'
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.04
        marker.color.r = 1.0
        marker.color.g = 0.2
        marker.color.b = 0.2
        marker.color.a = 0.85
        marker.points.extend([docking.position, tree.position])
        return marker


class Nav2Gui(QWidget):
    TERMINAL = {
        MissionStatus.MISSION_COMPLETED,
        MissionStatus.CANCELED,
        MissionStatus.FAILED,
    }
    ACTIVE = {
        MissionStatus.READY,
        MissionStatus.NAVIGATING,
        MissionStatus.VERIFYING_STOP,
        MissionStatus.ARM_SPRAYING,
        MissionStatus.PAUSED,
        MissionStatus.RETURNING_HOME,
        getattr(MissionStatus, 'DWELLING', -1),
    }

    def __init__(self, node):
        super().__init__()
        self.node = node
        self.editor = MissionEditor(getattr(
            node, 'default_arm_spray_duration_sec',
            DEFAULT_ARM_SPRAY_DURATION_SEC))
        self.save_directory = DEFAULT_SAVE_DIRECTORY
        self.candidate = None
        self.candidate_sequence = 0
        self.consumed_goal_sequence = 0
        self.pending_dock_pose = None
        self.pending_dock_sequence = 0
        self.pending = False
        self.required_initial_pose_sequence = 0
        # A fresh RViz 2D Estimate Pose is sufficient at first startup.  The
        # explicit re-localization flow below closes this gate again until AMCL
        # confirms its global reset.
        self.relocalization_ready = True
        self._build_ui()
        self.ros_timer = QTimer(self)
        self.ros_timer.timeout.connect(self._spin_ros)
        self.ros_timer.start(25)
        self._refresh()

    def _build_ui(self):
        self.setWindowTitle('WVCSC 导航喷洒控制器')
        self.setGeometry(180, 100, 1200, 780)
        layout = QVBoxLayout()

        task_layout = QGridLayout()
        self.record_start_button = QPushButton('确认当前起点')
        self.relocalize_button = QPushButton('重新定位并清空任务')
        self.point_type_combo = QComboBox()
        self.point_type_combo.addItem('通行点', POINT_TRANSIT)
        self.point_type_combo.addItem('病株检查点', POINT_INSPECT)
        self.point_type_combo.addItem('终点', POINT_FINISH)
        self.record_point_button = QPushButton('记录当前点')
        self.wide_spray_checkbox = QCheckBox('开启广域喷洒')
        self.wide_spray_checkbox.setToolTip(
            '驶向新点的来程区段开启；到达该点后自动关闭。')
        self.start_task_button = QPushButton('开始任务')
        task_layout.addWidget(self.record_start_button, 0, 0)
        task_layout.addWidget(QLabel('新点类型:'), 0, 1)
        task_layout.addWidget(self.point_type_combo, 0, 2)
        task_layout.addWidget(self.record_point_button, 0, 3)
        task_layout.addWidget(self.wide_spray_checkbox, 0, 4)
        task_layout.addWidget(self.start_task_button, 0, 5, 1, 2)
        task_layout.addWidget(self.relocalize_button, 0, 7)
        layout.addLayout(task_layout)

        self.candidate_label = QLabel('最新RViz机械臂基座点: 未收到 /manual_goal_pose')
        self.capture_label = QLabel('采集状态: 请选择点类型并点击 RViz 2D Goal')
        self.start_label = QLabel('起点: 未记录')
        self.return_home_checkbox = QCheckBox('完成后返回起点')
        self.image_toggle = QCheckBox('显示相机/YOLO画面')
        option_layout = QHBoxLayout()
        option_layout.addWidget(self.return_home_checkbox)
        option_layout.addWidget(self.image_toggle)
        option_layout.addStretch(1)
        layout.addWidget(self.candidate_label)
        layout.addWidget(self.capture_label)
        layout.addWidget(self.start_label)
        layout.addLayout(option_layout)

        self.wide_relay_label = None
        self.arm_relay_label = None
        if self.node.show_sim_spray_status:
            relay_layout = QHBoxLayout()
            self.wide_relay_label = QLabel('广域喷洒: ● 关闭')
            self.arm_relay_label = QLabel('喷嘴喷洒: ● 关闭')
            relay_layout.addWidget(self.wide_relay_label)
            relay_layout.addWidget(self.arm_relay_label)
            layout.addLayout(relay_layout)

        self.table = QTableWidget(0, 3)
        self.table.setHorizontalHeaderLabels(
            ['序号', '类型', '机械臂基座位姿 (x, y, yaw)'])
        self.table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.table.setSelectionMode(QAbstractItemView.SingleSelection)
        self.table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        layout.addWidget(self.table)

        edit_layout = QHBoxLayout()
        self.delete_button = QPushButton('删除选中点')
        self.up_button = QPushButton('上移')
        self.down_button = QPushButton('下移')
        self.clear_button = QPushButton('清空任务列表')
        for button in (self.delete_button, self.up_button,
                       self.down_button, self.clear_button):
            edit_layout.addWidget(button)
        layout.addLayout(edit_layout)

        control_layout = QHBoxLayout()
        self.abort_home_button = QPushButton('终止任务')
        self.abort_home_button.setStyleSheet('font-weight: bold;')
        self.home_button = QPushButton('返回起点')
        for button in (self.abort_home_button, self.home_button):
            control_layout.addWidget(button)
        layout.addLayout(control_layout)

        file_layout = QHBoxLayout()
        self.save_button = QPushButton('保存任务')
        self.load_button = QPushButton('加载任务')
        file_layout.addWidget(self.save_button)
        file_layout.addWidget(self.load_button)
        layout.addLayout(file_layout)

        self.status_label = QLabel('状态: 等待任务管理器')
        self.status_label.setAlignment(Qt.AlignCenter)
        self.log_area = QTextEdit()
        self.log_area.setReadOnly(True)
        manager_panel = QWidget()
        manager_layout = QVBoxLayout(manager_panel)
        manager_layout.addWidget(self.status_label)
        manager_layout.addWidget(self.log_area, 1)
        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(manager_panel)
        self.image_panel = RosImagePanel(self.node, active=False)
        self.image_panel.setVisible(False)
        splitter.addWidget(self.image_panel)
        splitter.setSizes([520, 780])
        self.output_splitter = splitter
        layout.addWidget(splitter, 1)
        self.setLayout(layout)

        self.record_start_button.clicked.connect(self._record_start)
        self.relocalize_button.clicked.connect(self._relocalize_and_clear)
        self.record_point_button.clicked.connect(self._record_point)
        self.point_type_combo.currentIndexChanged.connect(
            self._on_point_type_changed)
        self.start_task_button.clicked.connect(self._start_task)
        self.delete_button.clicked.connect(self._delete_point)
        self.up_button.clicked.connect(lambda: self._move_point(-1))
        self.down_button.clicked.connect(lambda: self._move_point(1))
        self.clear_button.clicked.connect(self._clear_points)
        self.abort_home_button.clicked.connect(self._abort_and_home)
        self.home_button.clicked.connect(lambda: self._trigger('return_home'))
        self.save_button.clicked.connect(self._save_dialog)
        self.load_button.clicked.connect(self._load_dialog)
        self.image_toggle.toggled.connect(self._set_image_panel_visible)

    def _spin_ros(self):
        try:
            rclpy.spin_once(self.node, timeout_sec=0.0)
        except RuntimeError:
            return
        self._refresh()

    def _set_image_panel_visible(self, visible):
        self.image_panel.setVisible(bool(visible))
        self.image_panel.set_active(bool(visible))
        if visible:
            self.output_splitter.setSizes([520, 780])

    def _update_record_point_button(self, *_args):
        if self.pending_dock_pose is not None:
            self.record_point_button.setText('记录树中心')
        elif self.point_type_combo.currentData() == POINT_INSPECT:
            self.record_point_button.setText('记录病株停靠位')
        else:
            self.record_point_button.setText('记录当前点')

    def _on_point_type_changed(self, *_args):
        if self.point_type_combo.currentData() == POINT_FINISH:
            self.wide_spray_checkbox.setChecked(False)
        self._update_record_point_button()

    def _refresh(self):
        if self.node.goal_sequence != self.candidate_sequence:
            self.candidate_sequence = self.node.goal_sequence
            self.candidate = self.node.latest_goal_pose
            if self.candidate is not None:
                self.candidate_label.setText(
                    '最新RViz机械臂基座点: '
                    f'x={self.candidate.position.x:.2f}, '
                    f'y={self.candidate.position.y:.2f}, '
                    f'yaw={pose_yaw(self.candidate):.2f}')
            self._publish_markers()
        state = self.node.status.state if self.node.status else None
        state_text = self.node.status.state_text if self.node.status else '等待任务管理器'
        if self.node.status and self.node.status.last_error:
            state_text += f' - {self.node.status.last_error}'
        self.status_label.setText(f'状态: {state_text}')
        wide_relay_label = getattr(self, 'wide_relay_label', None)
        arm_relay_label = getattr(self, 'arm_relay_label', None)
        if wide_relay_label is not None and arm_relay_label is not None:
            self._set_relay_label(
                wide_relay_label, '广域喷洒', self.node.sim_relay_active[1])
            self._set_relay_label(
                arm_relay_label, '喷嘴喷洒', self.node.sim_relay_active[2])
        busy = state in self.ACTIVE
        editable = not self.pending and not busy
        has_start = self.editor.start_pose is not None
        fresh_initial = (
            self.node.initial_pose_sequence > self.required_initial_pose_sequence)
        point_count = len(self.editor.points)
        self.record_start_button.setEnabled(
            editable and not has_start and self.relocalization_ready and fresh_initial)
        self.relocalize_button.setEnabled(editable)
        waiting_for_tree = self.pending_dock_pose is not None
        self.point_type_combo.setEnabled(editable and not waiting_for_tree)
        self.wide_spray_checkbox.setEnabled(
            editable and not waiting_for_tree and
            self.point_type_combo.currentData() != POINT_FINISH)
        required_sequence = (
            self.pending_dock_sequence if waiting_for_tree
            else self.consumed_goal_sequence)
        self.record_point_button.setEnabled(
            editable and self.candidate is not None
            and self.node.goal_sequence > required_sequence)
        self._update_record_point_button()
        self.start_task_button.setEnabled(
            editable and has_start and point_count >= 1)
        for button in (self.delete_button, self.up_button,
                       self.down_button, self.clear_button, self.save_button,
                       self.load_button):
            button.setEnabled(editable)
        self.abort_home_button.setEnabled(not self.pending)
        self.home_button.setEnabled(not self.pending and state in {
            MissionStatus.READY, MissionStatus.PAUSED,
            MissionStatus.VERIFYING_STOP, MissionStatus.MISSION_COMPLETED})
        if not has_start:
            if not self.relocalization_ready:
                self.start_label.setText('起点: 等待 AMCL 全局重定位服务成功')
            elif fresh_initial and self.node.latest_initial_pose is not None:
                pose = arm_anchor_from_vehicle_pose(
                    self.node.latest_initial_pose)
                self.start_label.setText(
                    '起点候选（机械臂基座）: RViz 已重新定位，等待确认 '
                    f'x={pose.position.x:.2f}, y={pose.position.y:.2f}, '
                    f'yaw={pose_yaw(pose):.2f}')

    @staticmethod
    def _set_relay_label(label, name, active):
        label.setText(f'{name}: ● ' + ('开启' if active else '关闭'))
        label.setStyleSheet(
            'color: #00d7d7; font-weight: bold;' if active
            else 'color: #808080;')

    def _record_start(self):
        if not self.relocalization_ready:
            self._log('记录起点失败：AMCL 全局重定位服务尚未成功')
            return
        if self.node.initial_pose_sequence <= self.required_initial_pose_sequence:
            self._log('记录起点失败：请先在 RViz 重新点击 2D Estimate Pose')
            return
        vehicle_pose = self.node.current_pose()
        if vehicle_pose is None:
            self._log('记录起点失败：请等待 AMCL 的 map→base TF 可用')
            return
        pose = arm_anchor_from_vehicle_pose(vehicle_pose)
        self.editor.start_pose = pose
        self.start_label.setText(
            f'起点（机械臂基座）: x={pose.position.x:.2f}, y={pose.position.y:.2f}, '
            f'yaw={pose_yaw(pose):.2f}')
        self._log('已确认当前起点：显示为机械臂基座，导航仍使用车辆基座')
        self._publish_markers()

    def _relocalize_and_clear(self):
        self.editor.start_pose = None
        self.editor.points.clear()
        self.candidate = None
        self.candidate_sequence = self.node.goal_sequence
        self.consumed_goal_sequence = self.node.goal_sequence
        self.pending_dock_pose = None
        self.pending_dock_sequence = self.node.goal_sequence
        self.required_initial_pose_sequence = self.node.initial_pose_sequence
        self.relocalization_ready = not bool(getattr(
            self.node, 'require_global_relocalization_service', True))
        self.start_label.setText(
            '起点: 正在请求 AMCL 全局重定位'
            if bool(getattr(self.node, 'require_global_relocalization_service', True))
            else '起点: 请在 RViz 点击 2D Estimate Pose 后确认')
        self.capture_label.setText('采集状态: 任务点已清空；请重新定位后设置任务点')
        self.candidate_label.setText('最新RViz机械臂基座点: 等待新的 2D Goal')
        self._update_table()
        self._publish_markers()

        if not bool(getattr(self.node, 'require_global_relocalization_service', True)):
            self._log('仿真未启动 AMCL 全局重定位服务；请在 RViz 点击 2D Estimate Pose 后确认起点')
            return

        client = self.node.service_clients['reinitialize_global_localization']
        if not client.service_is_ready():
            self._log('AMCL 全局重定位服务不可用；任务已清空，请稍后重新点击此按钮')
            return
        self.pending = True
        future = client.call_async(Empty.Request())

        def finished(done):
            self.pending = False
            try:
                done.result()
            except Exception as error:
                self._log(f'AMCL 全局重定位请求失败：{error}')
                return
            self.relocalization_ready = True
            self._log('AMCL 已全局重定位；请在 RViz 点击 2D Estimate Pose 后确认起点')

        future.add_done_callback(finished)

    def _record_point(self):
        if self.pending_dock_pose is not None:
            self._capture_tree_center()
            return
        self._add_point()

    def _add_point(self):
        if self.candidate is None:
            return
        point_type = self.point_type_combo.currentData()
        if point_type == POINT_INSPECT:
            self.pending_dock_pose = copy_pose(self.candidate)
            self.pending_dock_sequence = self.node.goal_sequence
            self.consumed_goal_sequence = self.node.goal_sequence
            self.capture_label.setText(
                '采集状态: 已记录病株机械臂基座停靠位；请在 RViz 点击树中心，再点击“记录树中心”')
            self._log('已记录病株机械臂基座停靠位，等待树中心点击')
            self._publish_markers()
            return

        self.editor.add_point(
            self.candidate,
            point_type=point_type,
            wide_spray_on_approach=self.wide_spray_checkbox.isChecked(),
            arm_spray_duration_sec=self.editor.spray_duration)
        self.consumed_goal_sequence = self.node.goal_sequence
        self._update_table()
        self._log(f'已添加{self._point_type_label(point_type)} {len(self.editor.points)}')
        self._publish_markers()

    def _capture_tree_center(self):
        if self.pending_dock_pose is None or self.candidate is None:
            return
        if self.node.goal_sequence <= self.pending_dock_sequence:
            self._log('请先在 RViz 点击新的树中心位置')
            return
        tree_x_m, tree_y_m = tree_offset_from_arm_anchor(
            self.pending_dock_pose, self.candidate)
        work_side = work_side_from_tree_y(tree_y_m)
        if work_side == WORK_SIDE_UNSPECIFIED:
            self._log('树中心与机械臂基座 Y 过近，无法判定左右侧；请重新点击树中心')
            return
        prospective = WorkPoint(
            pose=copy_pose(self.pending_dock_pose),
            tree_x_m=tree_x_m,
            tree_y_m=tree_y_m,
            tree_base_z_m=self.candidate.position.z,
            point_type=POINT_INSPECT,
            work_side=work_side,
            arm_spray_duration_sec=self.editor.spray_duration,
            tree_pose=copy_pose(self.candidate),
        )
        error = ik_recording_range_error(
            prospective, self.node.observation_mode,
            self.node.ik_recording_range_min_m,
            self.node.ik_recording_range_max_m)
        if error is None:
            error = simulation_parking_clearance_error(
                prospective,
                getattr(self.node, 'simulation_parking_clearance_check', False))
        if error is not None:
            # Do not leave a stale pending docking point that could later be
            # paired with an unrelated tree click.
            self.pending_dock_pose = None
            self.pending_dock_sequence = 0
            title = ('仿真停车位不合格'
                     if error.startswith('仿真停车位过近')
                     else 'IK 作业距离不合格')
            self.capture_label.setText(
                '采集状态: 停车位或IK作业距离不合格；请重新记录停靠位')
            self._log(f'拒绝病株点: {error}')
            QMessageBox.warning(self, title, error)
            self._publish_markers()
            return
        self.editor.add_point(
            self.pending_dock_pose,
            tree_x_m=tree_x_m,
            tree_y_m=tree_y_m,
            tree_base_z_m=self.candidate.position.z,
            point_type=POINT_INSPECT,
            work_side=work_side,
            wide_spray_on_approach=self.wide_spray_checkbox.isChecked(),
            arm_spray_duration_sec=self.editor.spray_duration,
            tree_pose=self.candidate)
        self.consumed_goal_sequence = self.node.goal_sequence
        self.pending_dock_pose = None
        self.pending_dock_sequence = 0
        self.capture_label.setText(
            '采集状态: 病株点已完成；可继续选择下一个点类型')
        self._update_table()
        message = (
            f'已添加病株点 {len(self.editor.points)}: '
            f'树相对基座=({tree_x_m:.2f}, {tree_y_m:.2f}) m, {work_side}')
        if getattr(self.node, 'simulation_parking_clearance_check', False):
            clearance = simulation_parking_clearance_m(
                self.editor.points[-1].pose, self.editor.points[-1].tree_pose)
            message += f'; 停车净余量={clearance:.2f} m'
        self._log(message)
        self._publish_markers()

    @staticmethod
    def _point_type_label(point_type):
        return {
            POINT_TRANSIT: '通行点',
            POINT_INSPECT: '病株检查点',
            POINT_FINISH: '终点',
        }[point_type]

    def _start_task(self):
        """提交当前列表；一个点和多个点共享同一条任务入口。"""
        points = [self._copy_work_point(point) for point in self.editor.points]
        self._submit_manual(points, 'task')

    @staticmethod
    def _copy_work_point(point):
        return WorkPoint(
            copy_pose(point.pose), point.tree_x_m, point.tree_y_m,
            point.tree_base_z_m, point.point_type, point.work_side,
            point.wide_spray_on_approach, point.arm_spray_duration_sec,
            point.dwell_time_sec,
            copy_pose(point.tree_pose) if point.tree_pose is not None else None)

    def _submit_manual(self, points, prefix):
        if self.editor.start_pose is None or not points:
            self._log('请先记录起点和作业目标')
            return
        self.editor.return_home_after_finish = self.return_home_checkbox.isChecked()
        self._log(f'路线预览: {route_timeline(points)}')
        if self.node.status and self.node.status.state in self.TERMINAL:
            self._trigger('reset', lambda _result: self._load_manual(points, prefix))
            return
        self._load_manual(points, prefix)

    def _load_manual(self, points, prefix):
        try:
            request = self.node.build_manual_request(
                self.editor.start_pose, points,
                self.editor.return_home_after_finish, prefix)
        except ValueError as error:
            self._log(f'任务数据无效: {error}')
            QMessageBox.warning(self, '任务数据无效', str(error))
            return
        self._request(
            'load', request,
            self._start_loaded_mission)

    def _start_loaded_mission(self, result):
        self._log(result.message)
        self._trigger('start')

    def _trigger(self, name, callback=None):
        self._request(name, Trigger.Request(), callback)

    def _abort_and_home(self):
        """Request the mission manager's serialized cancel-and-HOME flow."""
        self._log('正在终止导航与喷洒，并请求机械臂安全回 HOME')
        self._trigger('abort_and_home')

    def _request(self, name, request, callback=None):
        client = self.node.service_clients[name]
        if not client.service_is_ready():
            self._log(f'服务不可用: {client.srv_name}')
            return
        self.pending = True
        future = client.call_async(request)

        def finished(done):
            self.pending = False
            try:
                result = done.result()
            except Exception as error:
                self._log(f'服务调用失败: {error}')
                return
            if not result.success:
                self._log(result.message)
                return
            self._log(result.message)
            if callback is not None:
                callback(result)

        future.add_done_callback(finished)

    def _update_table(self):
        self.table.setRowCount(len(self.editor.points))
        for row, point in enumerate(self.editor.points):
            range_error = ik_recording_range_error(
                point, getattr(self.node, 'observation_mode', 'joint_presets'),
                getattr(self.node, 'ik_recording_range_min_m', 0.85),
                getattr(self.node, 'ik_recording_range_max_m', 1.45))
            if range_error is None:
                range_error = simulation_parking_clearance_error(
                    point,
                    getattr(self.node, 'simulation_parking_clearance_check', False))
            point_type = self._point_type_label(point.point_type)
            if point.wide_spray_on_approach:
                point_type += '（广域）'
            static_cells = (
                (0, QTableWidgetItem(str(row + 1))),
                (1, QTableWidgetItem(point_type)),
                (2, QTableWidgetItem(
                    f'({point.pose.position.x:.3f}, '
                    f'{point.pose.position.y:.3f}, '
                    f'{pose_yaw(point.pose):.3f})')),
            )
            for column, item in static_cells:
                if range_error is not None:
                    item.setBackground(QColor(255, 226, 226))
                    item.setToolTip(range_error)
                self.table.setItem(row, column, item)

    def _delete_point(self):
        row = self.table.currentRow()
        if 0 <= row < len(self.editor.points):
            self.editor.points.pop(row)
            self._update_table()
            self._publish_markers()

    def _move_point(self, offset):
        row = self.table.currentRow()
        target = row + offset
        if not (0 <= row < len(self.editor.points)
                and 0 <= target < len(self.editor.points)):
            return
        self.editor.points[row], self.editor.points[target] = (
            self.editor.points[target], self.editor.points[row])
        self._update_table()
        self.table.selectRow(target)
        self._publish_markers()

    def _clear_points(self):
        if not self.editor.points:
            return
        self.editor.points.clear()
        self.pending_dock_pose = None
        self.pending_dock_sequence = 0
        self.capture_label.setText('采集状态: 已清空列表')
        self._update_table()
        self._publish_markers()

    def _save_dialog(self):
        default_path = timestamped_mission_path(self.save_directory)
        path, _ = QFileDialog.getSaveFileName(
            self, '保存任务', default_path, 'JSON files (*.json)')
        if not path:
            return
        path = non_overwriting_json_path(path)
        try:
            self.editor.return_home_after_finish = self.return_home_checkbox.isChecked()
            self.editor.save(path)
        except (OSError, ValueError, KeyError) as error:
            QMessageBox.warning(self, '保存失败', str(error))
            return
        self.save_directory = os.path.dirname(path) or DEFAULT_SAVE_DIRECTORY
        self._log(f'已保存任务: {path}')

    def _load_dialog(self):
        path, _ = QFileDialog.getOpenFileName(
            self, '加载任务', self.save_directory, 'JSON files (*.json)')
        if not path:
            return
        try:
            self.editor.load(path)
        except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError) as error:
            QMessageBox.warning(self, '加载失败', str(error))
            return
        self.save_directory = os.path.dirname(path) or DEFAULT_SAVE_DIRECTORY
        self.return_home_checkbox.setChecked(self.editor.return_home_after_finish)
        if self.editor.start_pose is not None:
            self.start_label.setText(
                f'起点（机械臂基座）: x={self.editor.start_pose.position.x:.2f}, '
                f'y={self.editor.start_pose.position.y:.2f}, '
                f'yaw={pose_yaw(self.editor.start_pose):.2f}')
        else:
            self.required_initial_pose_sequence = self.node.initial_pose_sequence
            self.relocalization_ready = True
        self._update_table()
        self._publish_markers()
        self._log(f'已加载任务: {path}')
        if self.editor.load_warning:
            QMessageBox.warning(self, '旧任务已迁移', self.editor.load_warning)
            self._log(self.editor.load_warning)
        self._log('任务已加载，仅预览；请点击“开始任务”后提交导航与喷洒任务')

    def _publish_markers(self):
        self.node.publish_markers(
            self.editor, self.candidate, self.pending_dock_pose)

    def _log(self, message):
        timestamp = datetime.datetime.now().strftime('%H:%M:%S')
        self.log_area.append(f'[{timestamp}] {message}')

    def closeEvent(self, event):
        self.ros_timer.stop()
        super().closeEvent(event)


def main(args=None):
    remove_opencv_qt_plugin_override()
    rclpy.init(args=args)
    node = Nav2QtNode()
    app = QApplication(sys.argv)
    gui = Nav2Gui(node)
    gui.show()
    try:
        app.exec()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
