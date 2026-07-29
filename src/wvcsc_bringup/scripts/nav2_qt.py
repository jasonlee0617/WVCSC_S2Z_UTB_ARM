#!/usr/bin/env python3
"""Manual single-point and multi-point mission editor for WVCSC."""

import datetime
import json
import math
import os
import sys
import uuid

import rclpy
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
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
    ManualMissionPoint,
    MissionStatus,
)
from wvcsc_interfaces.srv import LoadManualMission
from wvcsc_bringup.mission_editor_model import (
    ARM_ANCHOR_POSE_REFERENCE,
    DEFAULT_ARM_SPRAY_DURATION_SEC,
    DEFAULT_SAVE_DIRECTORY,
    MAX_ARM_SPRAY_DURATION_SEC,
    MIN_ARM_SPRAY_DURATION_SEC,
    POINT_INSPECT,
    POINT_TRANSIT,
    SIM_NAV_MIN_PARKING_CLEARANCE_M,
    TREE_CANOPY_SEGMENTS,
    WorkPoint,
    MissionEditor,
    arm_anchor_from_vehicle_pose,
    copy_pose,
    non_overwriting_json_path,
    pose_yaw,
    route_timeline,
    simulation_parking_clearance_error,
    simulation_parking_clearance_m,
    timestamped_mission_path,
    tree_offset_from_arm_anchor,
    tree_offset_from_docking,
    valid_work_side,
    vehicle_pose_from_arm_anchor,
    work_side_from_tree_y,
    WORK_SIDE_LEFT,
    WORK_SIDE_RIGHT,
    WORK_SIDE_UNSPECIFIED,
)
from wvcsc_bringup.nav2_markers import ManualMissionMarkerBuilder
from wvcsc_bringup.qt_image_viewer import RosImagePanel


MANUAL_MISSION_REQUEST_FIELDS = (
    'header', 'mission_id', 'home_pose', 'return_home_after_mission',
    'points')


class ManualMissionInterfaceMismatchError(RuntimeError):
    """Raised when generated ROS interfaces do not match this Qt client."""


def manual_mission_request_contract_error(request):
    """Return a deployment error instead of letting a stale interface crash Qt."""
    missing = [
        field for field in MANUAL_MISSION_REQUEST_FIELDS
        if not hasattr(request, field)]
    if not missing:
        return None
    return (
        'LoadManualMission 接口版本不一致，当前 Request 缺少字段: '
        f'{", ".join(missing)}。请在同一工作区重建 wvcsc_interfaces、'
        'wvcsc_mission_manager 和 wvcsc_bringup，并重新 source install/setup.bash。')


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


class Nav2QtNode(Node):
    def __init__(self):
        super().__init__('nav2_qt')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('initial_pose_topic', '/initialpose')
        self.declare_parameter('goal_pose_topic', '/manual_goal_pose')
        self.declare_parameter('marker_topic', '/waypoints')
        self.declare_parameter('require_global_relocalization_service', True)
        self.declare_parameter('simulation_parking_clearance_check', False)
        self.declare_parameter(
            'default_arm_spray_duration_sec',
            DEFAULT_ARM_SPRAY_DURATION_SEC)
        self.declare_parameter('observation_mode', 'joint_presets')
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.require_global_relocalization_service = bool(
            self.get_parameter('require_global_relocalization_service').value)
        self.simulation_parking_clearance_check = bool(
            self.get_parameter('simulation_parking_clearance_check').value)
        self.default_arm_spray_duration_sec = float(
            self.get_parameter('default_arm_spray_duration_sec').value)
        self.observation_mode = str(
            self.get_parameter('observation_mode').value).strip().lower()
        if self.observation_mode not in {'ik', 'joint_presets'}:
            raise ValueError('observation_mode must be ik or joint_presets')
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
        self.spray_active = {'wide': None, 'nozzle': None}

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
        self.create_subscription(
            Bool, '/spray/wide_active',
            lambda message: self._on_spray_active('wide', message), latched)
        self.create_subscription(
            Bool, '/spray/simulated_active',
            lambda message: self._on_spray_active('nozzle', message), latched)
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

    def _on_spray_active(self, name, message):
        self.spray_active[name] = bool(message.data)

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
    def _point_constant(name, fallback):
        return int(getattr(ManualMissionPoint, name, fallback))

    def build_manual_request(self, start_pose, points,
                             return_home_after_mission, prefix):
        request = LoadManualMission.Request()
        contract_error = manual_mission_request_contract_error(request)
        if contract_error is not None:
            raise ManualMissionInterfaceMismatchError(contract_error)
        request.header.stamp = self.get_clock().now().to_msg()
        request.header.frame_id = self.map_frame
        request.mission_id = f'manual_{prefix}_{uuid.uuid4().hex[:8]}'
        request.home_pose = vehicle_pose_from_arm_anchor(start_pose)
        request.return_home_after_mission = return_home_after_mission
        for index, point in enumerate(points, start=1):
            error = valid_work_side(point)
            if error is None:
                error = simulation_parking_clearance_error(
                    point,
                    getattr(self, 'simulation_parking_clearance_check', False))
            if error is not None:
                raise ValueError(f'第 {index} 个点无效：{error}')
            route_point = ManualMissionPoint()
            route_point.point_id = f'{prefix}_{index:02d}'
            route_point.docking_pose = vehicle_pose_from_arm_anchor(point.pose)
            is_inspect = point.point_type == POINT_INSPECT
            route_point.tree_x_m = point.tree_x_m if is_inspect else 0.0
            route_point.tree_y_m = point.tree_y_m if is_inspect else 0.0
            route_point.tree_base_z_m = point.tree_base_z_m if is_inspect else 0.0
            route_point.spray_duration = (
                float(point.arm_spray_duration_sec)
                if is_inspect else 0.0)
            point_type = {
                POINT_INSPECT: self._point_constant('POINT_INSPECT', 0),
                POINT_TRANSIT: self._point_constant('POINT_TRANSIT', 1),
            }[point.point_type]
            work_side = {
                WORK_SIDE_UNSPECIFIED: self._point_constant(
                    'WORK_SIDE_UNSPECIFIED', 0),
                WORK_SIDE_LEFT: self._point_constant('WORK_SIDE_LEFT', 1),
                WORK_SIDE_RIGHT: self._point_constant('WORK_SIDE_RIGHT', 2),
            }[point.work_side]
            for name, value in (
                    ('point_type', point_type),
                    ('wide_spray_on_approach',
                     bool(point.wide_spray_on_approach)),
                    ('dwell_time_sec', float(point.dwell_time_sec)),
                    ('work_side', work_side),
                    ('arm_spray_duration_sec',
                     float(point.arm_spray_duration_sec) if is_inspect else 0.0)):
                setattr(route_point, name, value)
            request.points.append(route_point)
        return request

    def publish_markers(self, editor, candidate, pending_dock=None):
        self.marker_pub.publish(
            self._marker_builder().build(editor, candidate, pending_dock))

    def _marker_builder(self):
        return ManualMissionMarkerBuilder(
            self.map_frame, lambda: self.get_clock().now().to_msg())

    # Keep these private entry points during the Qt refactor.  They are used
    # by existing focused marker tests and preserve the former marker shapes.
    def _marker(self, *args):
        return ManualMissionMarkerBuilder(
            self.map_frame, lambda: self.get_clock().now().to_msg()).marker(*args)

    def _vehicle_marker(self, *args):
        return ManualMissionMarkerBuilder(
            self.map_frame, lambda: self.get_clock().now().to_msg()).vehicle_marker(
                *args)

    def _mount_line(self, *args):
        return ManualMissionMarkerBuilder(
            self.map_frame, lambda: self.get_clock().now().to_msg()).mount_line(*args)

    def _vehicle_route_marker(self, *args):
        return ManualMissionMarkerBuilder(
            self.map_frame, lambda: self.get_clock().now().to_msg()).vehicle_route_marker(
                *args)

    def _label(self, *args):
        return ManualMissionMarkerBuilder(
            self.map_frame, lambda: self.get_clock().now().to_msg()).label(*args)

    def _tree_root_marker(self, *args):
        return ManualMissionMarkerBuilder(
            self.map_frame, lambda: self.get_clock().now().to_msg()).tree_root_marker(
                *args)

    def _tree_canopy_marker(self, *args):
        return ManualMissionMarkerBuilder(
            self.map_frame, lambda: self.get_clock().now().to_msg()).tree_canopy_marker(
                *args)

    def _tree_distance_label(self, *args):
        return ManualMissionMarkerBuilder(
            self.map_frame, lambda: self.get_clock().now().to_msg()).tree_distance_label(
                *args)

    def _tree_label(self, *args):
        return ManualMissionMarkerBuilder(
            self.map_frame, lambda: self.get_clock().now().to_msg()).tree_label(*args)

    def _tree_line(self, *args):
        return ManualMissionMarkerBuilder(
            self.map_frame, lambda: self.get_clock().now().to_msg()).tree_line(*args)


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

        relay_layout = QHBoxLayout()
        self.wide_relay_label = QLabel('广域喷洒: ● 未收到状态')
        self.arm_relay_label = QLabel('喷嘴喷洒: ● 未收到状态')
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
        self._set_spray_label(
            self.wide_relay_label, '广域喷洒',
            self.node.spray_active['wide'], '#1e88e5')
        self._set_spray_label(
            self.arm_relay_label, '喷嘴喷洒',
            self.node.spray_active['nozzle'], '#e53935')
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
            editable and not waiting_for_tree)
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
    def _set_spray_label(label, name, active, active_color):
        if active is None:
            label.setText(f'{name}: ● 未收到状态')
            label.setStyleSheet('color: #808080;')
            return
        label.setText(f'{name}: ● ' + ('开启' if active else '关闭'))
        label.setStyleSheet(
            f'color: {active_color}; font-weight: bold;'
            if active else 'color: #808080;')

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
        error = simulation_parking_clearance_error(
            prospective,
            getattr(self.node, 'simulation_parking_clearance_check', False))
        if error is not None:
            # Do not leave a stale pending docking point that could later be
            # paired with an unrelated tree click.
            self.pending_dock_pose = None
            self.pending_dock_sequence = 0
            title = '仿真停车位不合格'
            self.capture_label.setText(
                '采集状态: 停车位不合格；请重新记录停靠位')
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
        self.editor.return_home_after_mission = self.return_home_checkbox.isChecked()
        self._log(f'路线预览: {route_timeline(points)}')
        if self.node.status and self.node.status.state in self.TERMINAL:
            self._trigger('reset', lambda _result: self._load_manual(points, prefix))
            return
        self._load_manual(points, prefix)

    def _load_manual(self, points, prefix):
        try:
            request = self.node.build_manual_request(
                self.editor.start_pose, points,
                self.editor.return_home_after_mission, prefix)
        except (ValueError, ManualMissionInterfaceMismatchError) as error:
            title = (
                'ROS 接口版本不一致'
                if isinstance(error, ManualMissionInterfaceMismatchError)
                else '任务数据无效')
            self._log(f'{title}: {error}')
            QMessageBox.warning(self, title, str(error))
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
            range_error = simulation_parking_clearance_error(
                point,
                getattr(self.node, 'simulation_parking_clearance_check', False))
            point_type = self._point_type_label(point.point_type)
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
            self.editor.return_home_after_mission = self.return_home_checkbox.isChecked()
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
        self.return_home_checkbox.setChecked(self.editor.return_home_after_mission)
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
