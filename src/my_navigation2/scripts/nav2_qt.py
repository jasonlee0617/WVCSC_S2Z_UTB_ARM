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
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QApplication,
    QAbstractItemView,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
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
    QVBoxLayout,
    QWidget,
)
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray
from wvcsc_interfaces.msg import (
    ManualMissionTarget,
    MissionPlan,
    MissionStatus,
)
from wvcsc_interfaces.srv import LoadManualMission


DEFAULT_SAVE_PATH = os.path.expanduser('~/navigation_points.json')

# These values are the installed ``alicia_mount_joint`` transform.  The
# Qt editor deliberately uses the same fixed geometry as the real route
# capture tools: a tree click is converted into alicia_base_link coordinates,
# not vehicle-base coordinates.
ARM_BASE_FORWARD_OFFSET_M = -0.40
ARM_BASE_LEFT_OFFSET_M = 0.0
ARM_BASE_YAW_RAD = math.pi
SIDE_EPSILON_M = 0.05

POINT_INSPECT = 'INSPECT'
POINT_TRANSIT = 'TRANSIT'
POINT_FINISH = 'FINISH'
POINT_TYPES = (POINT_TRANSIT, POINT_INSPECT, POINT_FINISH)

WORK_SIDE_UNSPECIFIED = 'UNSPECIFIED'
WORK_SIDE_LEFT = 'LEFT'
WORK_SIDE_RIGHT = 'RIGHT'


def tree_offset_from_docking(docking_pose, tree_pose):
    """Return signed alicia_base_link XY for a manually clicked tree.

    ``docking_pose`` and ``tree_pose`` are both in ``map``.  The arm is
    mounted at (-0.40, 0.0, pi) relative to the vehicle base.
    """
    yaw = pose_yaw(docking_pose)
    cosine, sine = math.cos(yaw), math.sin(yaw)
    arm_x = (
        docking_pose.position.x + cosine * ARM_BASE_FORWARD_OFFSET_M
        - sine * ARM_BASE_LEFT_OFFSET_M)
    arm_y = (
        docking_pose.position.y + sine * ARM_BASE_FORWARD_OFFSET_M
        + cosine * ARM_BASE_LEFT_OFFSET_M)
    arm_yaw = yaw + ARM_BASE_YAW_RAD
    arm_cosine, arm_sine = math.cos(arm_yaw), math.sin(arm_yaw)
    dx = tree_pose.position.x - arm_x
    dy = tree_pose.position.y - arm_y
    return (
        arm_cosine * dx + arm_sine * dy,
        -arm_sine * dx + arm_cosine * dy,
    )


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
    arm_spray_duration_sec: float = 3.0
    dwell_time_sec: float = 0.0
    tree_pose: Pose | None = None


class MissionEditor:
    schema_version = 4

    def __init__(self):
        self.start_pose = None
        self.points = []
        self.spray_duration = 2.0
        self.return_home_after_finish = False
        self.load_warning = None

    def add_point(self, pose, tree_x_m=0.0, tree_y_m=0.0, tree_base_z_m=0.0,
                  point_type=POINT_TRANSIT,
                  work_side=WORK_SIDE_UNSPECIFIED,
                  wide_spray_on_approach=False,
                  arm_spray_duration_sec=3.0,
                  dwell_time_sec=0.0,
                  tree_pose=None):
        if point_type not in POINT_TYPES:
            raise ValueError(f'unsupported point type: {point_type}')
        if point_type == POINT_INSPECT and work_side == WORK_SIDE_UNSPECIFIED:
            work_side = work_side_from_tree_y(tree_y_m)
        if point_type == POINT_FINISH:
            wide_spray_on_approach = False
        self.points.append(WorkPoint(
            copy_pose(pose), float(tree_x_m), float(tree_y_m),
            float(tree_base_z_m), point_type, work_side,
            bool(wide_spray_on_approach), float(arm_spray_duration_sec),
            float(dwell_time_sec),
            copy_pose(tree_pose) if tree_pose is not None else None))

    def save(self, path):
        data = {
            'schema_version': self.schema_version,
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
        if version not in (3, self.schema_version):
            raise ValueError(
                f'unsupported navigation file schema_version: {version!r}')
        self.start_pose = (
            pose_from_json(data['start_pose'])
            if data.get('start_pose') else None)
        self.spray_duration = float(data.get('spray_duration', 2.0))
        self.return_home_after_finish = bool(
            data.get('return_home_after_finish', False))
        self.load_warning = None
        if version == 3:
            # Schema v3 represented every point as an arm target.  Preserve
            # its measured offsets, but make the migration visible because it
            # does not contain the manually clicked map tree centre or route
            # operation fields.
            raw_points = data.get('targets', [])
            self.points = [
                WorkPoint(
                    pose_from_json(item['pose']),
                    float(item['tree_x_m']),
                    float(item['tree_y_m']),
                    float(item.get('tree_base_z_m', 0.0)),
                    POINT_INSPECT,
                    work_side_from_tree_y(item['tree_y_m']),
                    False,
                    float(data.get('spray_duration', 2.0)),
                    0.0,
                    None,
                )
                for item in raw_points
            ]
            self.load_warning = (
                '已导入旧版任务：所有点均按病株检查点处理；请复核点类型、'
                '广域喷洒和侧位。旧文件没有树中心地图坐标。')
            return

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
                (False if point_type == POINT_FINISH else
                 bool(item.get('wide_spray_on_approach', False))),
                float(item.get('arm_spray_duration_sec',
                               data.get('spray_duration', 2.0))),
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
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.latest_initial_pose = None
        self.latest_goal_pose = None
        self.goal_sequence = 0
        self.status = None
        self.plan = None

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
        self.create_subscription(MissionPlan, '/mission/plan',
                                 self._on_plan, latched)
        self.marker_pub = self.create_publisher(
            MarkerArray, str(self.get_parameter('marker_topic').value), 10)
        self.service_clients = {
            'load': self.create_client(LoadManualMission,
                                       '/mission/load_manual'),
            'start': self.create_client(Trigger, '/mission/start'),
            'pause': self.create_client(Trigger, '/mission/pause'),
            'resume': self.create_client(Trigger, '/mission/resume'),
            'skip': self.create_client(Trigger, '/mission/skip_current'),
            'cancel': self.create_client(Trigger, '/mission/cancel'),
            'abort_and_home': self.create_client(
                Trigger, '/mission/abort_and_home'),
            'return_home': self.create_client(Trigger, '/mission/return_home'),
            'reset': self.create_client(Trigger, '/mission/reset'),
        }

    def _on_initial_pose(self, message):
        if message.header.frame_id == self.map_frame:
            self.latest_initial_pose = copy_pose(message.pose.pose)

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

    def _on_plan(self, message):
        self.plan = message

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

    def build_manual_request(self, start_pose, points, spray_duration,
                             return_home_after_finish, prefix):
        request = LoadManualMission.Request()
        request.header.stamp = self.get_clock().now().to_msg()
        request.header.frame_id = self.map_frame
        request.mission_id = f'manual_{prefix}_{uuid.uuid4().hex[:8]}'
        request.home_pose = copy_pose(start_pose)
        request.return_home_after_finish = return_home_after_finish
        for index, point in enumerate(points, start=1):
            error = valid_work_side(point)
            if error is not None:
                raise ValueError(f'第 {index} 个点无效：{error}')
            target = ManualMissionTarget()
            target.target_id = f'{prefix}_{index:02d}'
            target.docking_pose = copy_pose(point.pose)
            is_inspect = point.point_type == POINT_INSPECT
            target.tree_x_m = point.tree_x_m if is_inspect else 0.0
            target.tree_y_m = point.tree_y_m if is_inspect else 0.0
            target.tree_base_z_m = point.tree_base_z_m if is_inspect else 0.0
            target.use_tree_offset_from_arm_base = is_inspect
            target.spray_duration = (
                float(point.arm_spray_duration_sec)
                if is_inspect else 0.0)
            target.confidence = 1.0
            target.evidence_uri = 'manual://rviz'
            target.compute_docking_pose = False
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
        for index, point in enumerate(editor.points, start=1):
            color = {
                POINT_TRANSIT: (0.1, 0.6, 1.0),
                POINT_INSPECT: (1.0, 0.75, 0.0),
                POINT_FINISH: (0.15, 0.85, 0.3),
            }[point.point_type]
            markers.markers.append(self._marker(
                point.pose, 'manual_target', index, *color))
            markers.markers.append(self._label(point.pose, index, point))
            if point.tree_pose is not None:
                markers.markers.append(self._tree_marker(point.tree_pose, index))
                markers.markers.append(
                    self._tree_line(point.pose, point.tree_pose, index))
        if candidate is not None:
            markers.markers.append(self._marker(
                candidate, 'manual_candidate', 1000, 1.0, 0.8, 0.0))
        if pending_dock is not None:
            markers.markers.append(self._marker(
                pending_dock, 'manual_pending_inspect_dock', 1001,
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

    def _tree_marker(self, pose, marker_id):
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'manual_tree_center'
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = copy_pose(pose)
        marker.scale.x = marker.scale.y = marker.scale.z = 0.24
        marker.color.r = 1.0
        marker.color.g = 0.1
        marker.color.b = 0.1
        marker.color.a = 0.9
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
        self.editor = MissionEditor()
        self.save_path = DEFAULT_SAVE_PATH
        self.candidate = None
        self.candidate_sequence = 0
        self.consumed_goal_sequence = 0
        self.pending_dock_pose = None
        self.pending_dock_sequence = 0
        self.single_mission_id = None
        self.pending = False
        self._build_ui()
        self.ros_timer = QTimer(self)
        self.ros_timer.timeout.connect(self._spin_ros)
        self.ros_timer.start(25)
        self._refresh()

    def _build_ui(self):
        self.setWindowTitle('WVCSC 导航喷洒控制器')
        self.setGeometry(220, 160, 1220, 720)
        layout = QVBoxLayout()

        task_layout = QGridLayout()
        self.record_start_button = QPushButton('记录起点')
        self.point_type_combo = QComboBox()
        self.point_type_combo.addItem('通行点', POINT_TRANSIT)
        self.point_type_combo.addItem('病株检查点', POINT_INSPECT)
        self.point_type_combo.addItem('终点', POINT_FINISH)
        self.add_point_button = QPushButton('使用最新目标为停靠位')
        self.capture_tree_button = QPushButton('使用下一目标为树中心')
        self.single_button = QPushButton('单点导航+喷洒')
        self.multi_button = QPushButton('多点导航+喷洒')
        task_layout.addWidget(self.record_start_button, 0, 0)
        task_layout.addWidget(QLabel('新点类型:'), 0, 1)
        task_layout.addWidget(self.point_type_combo, 0, 2)
        task_layout.addWidget(self.add_point_button, 0, 3)
        task_layout.addWidget(self.capture_tree_button, 0, 4)
        task_layout.addWidget(self.single_button, 0, 5)
        task_layout.addWidget(self.multi_button, 0, 6)
        task_layout.addWidget(QLabel('病株默认喷洒时长 (s):'), 1, 0)
        self.duration_spin = QDoubleSpinBox()
        self.duration_spin.setRange(0.2, 10.0)
        self.duration_spin.setSingleStep(0.1)
        self.duration_spin.setValue(self.editor.spray_duration)
        task_layout.addWidget(self.duration_spin, 1, 1)
        task_layout.addWidget(QLabel('默认停留 (s):'), 1, 2)
        self.dwell_spin = QDoubleSpinBox()
        self.dwell_spin.setRange(0.0, 60.0)
        self.dwell_spin.setSingleStep(0.5)
        task_layout.addWidget(self.dwell_spin, 1, 3)
        self.wide_spray_checkbox = QCheckBox('驶向该点时开启广域喷洒')
        task_layout.addWidget(self.wide_spray_checkbox, 1, 4, 1, 3)
        layout.addLayout(task_layout)

        self.candidate_label = QLabel('最新RViz终点: 未收到 /manual_goal_pose')
        self.capture_label = QLabel('采集状态: 请选择点类型并点击 RViz 2D Goal')
        self.start_label = QLabel('起点: 未记录')
        self.return_home_checkbox = QCheckBox('完成后返回起点')
        layout.addWidget(self.candidate_label)
        layout.addWidget(self.capture_label)
        layout.addWidget(self.start_label)
        layout.addWidget(self.return_home_checkbox)

        self.table = QTableWidget(0, 11)
        self.table.setHorizontalHeaderLabels(
            ['序号', '类型', '停靠 X (m)', '停靠 Y (m)', 'yaw (rad)',
             '树相对基座 X (m)', '树相对基座 Y (m)', '侧位',
             '驶向本点广域喷洒', '病株喷洒(s)', '停留(s)'])
        self.table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.table.setSelectionMode(QAbstractItemView.SingleSelection)
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        layout.addWidget(self.table)

        edit_layout = QHBoxLayout()
        self.delete_button = QPushButton('删除选中点')
        self.up_button = QPushButton('上移')
        self.down_button = QPushButton('下移')
        self.clear_button = QPushButton('清空多点列表')
        for button in (self.delete_button, self.up_button,
                       self.down_button, self.clear_button):
            edit_layout.addWidget(button)
        layout.addLayout(edit_layout)

        control_layout = QHBoxLayout()
        self.pause_button = QPushButton('暂停')
        self.resume_button = QPushButton('继续')
        self.skip_button = QPushButton('跳过当前')
        self.cancel_button = QPushButton('取消任务')
        self.abort_home_button = QPushButton('终止作业并回HOME')
        self.home_button = QPushButton('返回起点')
        self.reset_button = QPushButton('重置任务')
        for button in (self.pause_button, self.resume_button, self.skip_button,
                       self.cancel_button, self.abort_home_button,
                       self.home_button, self.reset_button):
            control_layout.addWidget(button)
        layout.addLayout(control_layout)

        file_layout = QHBoxLayout()
        self.save_button = QPushButton('保存多点任务')
        self.load_button = QPushButton('加载多点任务')
        file_layout.addWidget(self.save_button)
        file_layout.addWidget(self.load_button)
        layout.addLayout(file_layout)

        self.status_label = QLabel('状态: 等待任务管理器')
        self.status_label.setAlignment(Qt.AlignCenter)
        self.log_area = QTextEdit()
        self.log_area.setReadOnly(True)
        layout.addWidget(self.status_label)
        layout.addWidget(self.log_area)
        self.setLayout(layout)

        self.record_start_button.clicked.connect(self._record_start)
        self.add_point_button.clicked.connect(self._add_point)
        self.capture_tree_button.clicked.connect(self._capture_tree_center)
        self.single_button.clicked.connect(self._start_single)
        self.multi_button.clicked.connect(self._start_multi)
        self.delete_button.clicked.connect(self._delete_point)
        self.up_button.clicked.connect(lambda: self._move_point(-1))
        self.down_button.clicked.connect(lambda: self._move_point(1))
        self.clear_button.clicked.connect(self._clear_points)
        self.pause_button.clicked.connect(lambda: self._trigger('pause'))
        self.resume_button.clicked.connect(lambda: self._trigger('resume'))
        self.skip_button.clicked.connect(lambda: self._trigger('skip'))
        self.cancel_button.clicked.connect(lambda: self._trigger('cancel'))
        self.abort_home_button.clicked.connect(self._abort_and_home)
        self.home_button.clicked.connect(lambda: self._trigger('return_home'))
        self.reset_button.clicked.connect(lambda: self._trigger('reset'))
        self.save_button.clicked.connect(self._save_dialog)
        self.load_button.clicked.connect(self._load_dialog)

    def _spin_ros(self):
        try:
            rclpy.spin_once(self.node, timeout_sec=0.0)
        except RuntimeError:
            return
        self._refresh()

    def _refresh(self):
        self._consume_completed_single_target()
        if self.node.goal_sequence != self.candidate_sequence:
            self.candidate_sequence = self.node.goal_sequence
            self.candidate = self.node.latest_goal_pose
            if self.candidate is not None:
                self.candidate_label.setText(
                    '最新RViz终点: '
                    f'x={self.candidate.position.x:.2f}, '
                    f'y={self.candidate.position.y:.2f}, '
                    f'yaw={pose_yaw(self.candidate):.2f}')
            self._publish_markers()
        state = self.node.status.state if self.node.status else None
        state_text = self.node.status.state_text if self.node.status else '等待任务管理器'
        if self.node.status and self.node.status.last_error:
            state_text += f' - {self.node.status.last_error}'
        self.status_label.setText(f'状态: {state_text}')
        busy = state in self.ACTIVE
        editable = not self.pending and not busy
        has_start = self.editor.start_pose is not None
        point_count = len(self.editor.points)
        self.record_start_button.setEnabled(editable)
        self.point_type_combo.setEnabled(editable)
        self.add_point_button.setEnabled(
            editable and self.candidate is not None
            and self.node.goal_sequence > self.consumed_goal_sequence)
        self.capture_tree_button.setEnabled(
            editable and self.pending_dock_pose is not None
            and self.candidate is not None
            and self.node.goal_sequence > self.pending_dock_sequence)
        self.single_button.setEnabled(
            editable and has_start and point_count == 1)
        self.multi_button.setEnabled(
            editable and has_start and point_count >= 2)
        for button in (self.delete_button, self.up_button,
                       self.down_button, self.clear_button, self.save_button,
                       self.load_button):
            button.setEnabled(editable)
        self.pause_button.setEnabled(not self.pending and state == MissionStatus.NAVIGATING)
        self.resume_button.setEnabled(not self.pending and state == MissionStatus.PAUSED)
        self.skip_button.setEnabled(not self.pending and state in {
            MissionStatus.READY, MissionStatus.PAUSED,
            MissionStatus.VERIFYING_STOP, MissionStatus.ARM_SPRAYING,
            getattr(MissionStatus, 'DWELLING', -1)})
        self.cancel_button.setEnabled(not self.pending and state in self.ACTIVE)
        self.abort_home_button.setEnabled(not self.pending)
        self.home_button.setEnabled(not self.pending and state in {
            MissionStatus.READY, MissionStatus.PAUSED,
            MissionStatus.VERIFYING_STOP, MissionStatus.MISSION_COMPLETED})
        self.reset_button.setEnabled(not self.pending and state in self.TERMINAL)

    def _record_start(self):
        pose = self.node.latest_initial_pose or self.node.current_pose()
        if pose is None:
            self._log('记录起点失败：请先在RViz点击 2D Estimate Pose，或确认TF可用')
            return
        self.editor.start_pose = copy_pose(pose)
        self.start_label.setText(
            f'起点: x={pose.position.x:.2f}, y={pose.position.y:.2f}, '
            f'yaw={pose_yaw(pose):.2f}')
        self._log('已记录起点')
        self._publish_markers()

    def _add_point(self):
        if self.candidate is None:
            return
        point_type = self.point_type_combo.currentData()
        if point_type == POINT_INSPECT:
            self.pending_dock_pose = copy_pose(self.candidate)
            self.pending_dock_sequence = self.node.goal_sequence
            self.consumed_goal_sequence = self.node.goal_sequence
            self.capture_label.setText(
                '采集状态: 已记录病株停车位；请在 RViz 点击树中心，再点击“使用下一目标为树中心”')
            self._log('已记录病株停车位，等待树中心点击')
            self._publish_markers()
            return

        self.editor.add_point(
            self.candidate,
            point_type=point_type,
            wide_spray_on_approach=self.wide_spray_checkbox.isChecked(),
            arm_spray_duration_sec=self.duration_spin.value(),
            dwell_time_sec=self.dwell_spin.value())
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
        tree_x_m, tree_y_m = tree_offset_from_docking(
            self.pending_dock_pose, self.candidate)
        work_side = work_side_from_tree_y(tree_y_m)
        if work_side == WORK_SIDE_UNSPECIFIED:
            self._log('树中心与机械臂基座 Y 过近，无法判定左右侧；请重新点击树中心')
            return
        self.editor.add_point(
            self.pending_dock_pose,
            tree_x_m=tree_x_m,
            tree_y_m=tree_y_m,
            tree_base_z_m=self.candidate.position.z,
            point_type=POINT_INSPECT,
            work_side=work_side,
            wide_spray_on_approach=self.wide_spray_checkbox.isChecked(),
            arm_spray_duration_sec=self.duration_spin.value(),
            dwell_time_sec=self.dwell_spin.value(),
            tree_pose=self.candidate)
        self.consumed_goal_sequence = self.node.goal_sequence
        self.pending_dock_pose = None
        self.pending_dock_sequence = 0
        self.capture_label.setText(
            '采集状态: 病株点已完成；可继续选择下一个点类型')
        self._update_table()
        self._log(
            f'已添加病株点 {len(self.editor.points)}: '
            f'树相对基座=({tree_x_m:.2f}, {tree_y_m:.2f}) m, {work_side}')
        self._publish_markers()

    @staticmethod
    def _point_type_label(point_type):
        return {
            POINT_TRANSIT: '通行点',
            POINT_INSPECT: '病株检查点',
            POINT_FINISH: '终点',
        }[point_type]

    def _start_single(self):
        if len(self.editor.points) != 1:
            return
        self._submit_manual([self._copy_work_point(self.editor.points[0])],
                            'single')

    def _start_multi(self):
        points = [self._copy_work_point(point) for point in self.editor.points]
        self._submit_manual(points, 'multi')

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
        self.editor.spray_duration = self.duration_spin.value()
        self.editor.return_home_after_finish = self.return_home_checkbox.isChecked()
        if self.node.status and self.node.status.state in self.TERMINAL:
            self._trigger('reset', lambda _result: self._load_manual(points, prefix))
            return
        self._load_manual(points, prefix)

    def _load_manual(self, points, prefix):
        try:
            request = self.node.build_manual_request(
                self.editor.start_pose, points, self.editor.spray_duration,
                self.editor.return_home_after_finish, prefix)
        except ValueError as error:
            self._log(f'任务数据无效: {error}')
            return
        self._request(
            'load', request,
            lambda result: self._start_loaded_mission(
                result, request.mission_id if prefix == 'single' else None))

    def _start_loaded_mission(self, result, single_mission_id=None):
        if single_mission_id is not None:
            self.single_mission_id = single_mission_id
        self._log(result.message)
        self._trigger('start')

    def _consume_completed_single_target(self):
        status = self.node.status
        if (self.single_mission_id is None or status is None
                or status.mission_id != self.single_mission_id
                or status.completed_targets < 1):
            return
        self.single_mission_id = None
        if self.editor.points:
            self.editor.points.pop(0)
            self._update_table()
            self._publish_markers()
            self._log('单点喷洒完成，已从列表删除')

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
            self.table.setItem(row, 0, QTableWidgetItem(str(row + 1)))
            self.table.setCellWidget(
                row, 1, self._point_type_combo(point))
            self.table.setItem(row, 2, QTableWidgetItem(f'{point.pose.position.x:.3f}'))
            self.table.setItem(row, 3, QTableWidgetItem(f'{point.pose.position.y:.3f}'))
            self.table.setItem(row, 4, QTableWidgetItem(f'{pose_yaw(point.pose):.3f}'))
            self.table.setCellWidget(
                row, 5, self._offset_spin(point.tree_x_m, point, 'tree_x_m'))
            self.table.setCellWidget(
                row, 6, self._offset_spin(point.tree_y_m, point, 'tree_y_m'))
            self.table.setCellWidget(row, 7, self._work_side_combo(point))
            self.table.setCellWidget(row, 8, self._wide_spray_checkbox(point))
            self.table.setCellWidget(
                row, 9, self._duration_cell(point, 'arm_spray_duration_sec',
                                             0.2, 10.0, 0.1))
            self.table.setCellWidget(
                row, 10, self._duration_cell(point, 'dwell_time_sec',
                                              0.0, 60.0, 0.5))

    def _point_type_combo(self, point):
        combo = QComboBox()
        for label, value in (
                ('通行点', POINT_TRANSIT),
                ('病株检查点', POINT_INSPECT),
                ('终点', POINT_FINISH)):
            combo.addItem(label, value)
        combo.setCurrentIndex(combo.findData(point.point_type))

        def changed(_index):
            point.point_type = combo.currentData()
            if point.point_type == POINT_INSPECT:
                inferred = work_side_from_tree_y(point.tree_y_m)
                if inferred != WORK_SIDE_UNSPECIFIED:
                    point.work_side = inferred
            elif point.point_type == POINT_FINISH:
                point.wide_spray_on_approach = False
                QTimer.singleShot(0, self._update_table)
            self._publish_markers()

        combo.currentIndexChanged.connect(changed)
        return combo

    def _work_side_combo(self, point):
        combo = QComboBox()
        for label, value in (
                ('未指定', WORK_SIDE_UNSPECIFIED),
                ('左侧 (+Y)', WORK_SIDE_LEFT),
                ('右侧 (-Y)', WORK_SIDE_RIGHT)):
            combo.addItem(label, value)
        combo.setCurrentIndex(combo.findData(point.work_side))
        combo.currentIndexChanged.connect(
            lambda _index: setattr(point, 'work_side', combo.currentData()))
        return combo

    def _wide_spray_checkbox(self, point):
        checkbox = QCheckBox()
        checkbox.setChecked(
            point.wide_spray_on_approach
            if point.point_type != POINT_FINISH else False)
        checkbox.setEnabled(point.point_type != POINT_FINISH)
        checkbox.toggled.connect(
            lambda checked: setattr(point, 'wide_spray_on_approach',
                                    bool(checked)))
        return checkbox

    @staticmethod
    def _duration_cell(point, attribute, minimum, maximum, step):
        spin = QDoubleSpinBox()
        spin.setRange(minimum, maximum)
        spin.setDecimals(2)
        spin.setSingleStep(step)
        spin.setValue(float(getattr(point, attribute)))
        spin.valueChanged.connect(
            lambda current: setattr(point, attribute, float(current)))
        return spin

    def _offset_spin(self, value, target, attribute):
        spin = QDoubleSpinBox()
        spin.setRange(-10.0, 10.0)
        spin.setDecimals(3)
        spin.setSingleStep(0.05)
        spin.setValue(value)
        def changed(current):
            setattr(target, attribute, float(current))
            self._publish_markers()
        spin.valueChanged.connect(changed)
        return spin

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
        path, _ = QFileDialog.getSaveFileName(
            self, '保存多点任务', self.save_path, 'JSON files (*.json)')
        if not path:
            return
        try:
            self.editor.spray_duration = self.duration_spin.value()
            self.editor.return_home_after_finish = self.return_home_checkbox.isChecked()
            self.editor.save(path)
        except (OSError, ValueError, KeyError) as error:
            QMessageBox.warning(self, '保存失败', str(error))
            return
        self.save_path = path
        self._log(f'已保存任务: {path}')

    def _load_dialog(self):
        path, _ = QFileDialog.getOpenFileName(
            self, '加载多点任务', self.save_path, 'JSON files (*.json)')
        if not path:
            return
        try:
            self.editor.load(path)
        except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError) as error:
            QMessageBox.warning(self, '加载失败', str(error))
            return
        self.save_path = path
        self.duration_spin.setValue(self.editor.spray_duration)
        self.return_home_checkbox.setChecked(self.editor.return_home_after_finish)
        if self.editor.start_pose is not None:
            self.start_label.setText(
                f'起点: x={self.editor.start_pose.position.x:.2f}, '
                f'y={self.editor.start_pose.position.y:.2f}, '
                f'yaw={pose_yaw(self.editor.start_pose):.2f}')
        self._update_table()
        self._publish_markers()
        self._log(f'已加载任务: {path}')
        if self.editor.load_warning:
            QMessageBox.warning(self, '旧任务已迁移', self.editor.load_warning)
            self._log(self.editor.load_warning)

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
