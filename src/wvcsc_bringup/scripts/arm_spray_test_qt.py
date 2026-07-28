#!/usr/bin/env python3
"""Single-target Alicia-M spray test with automatic HOME recovery readiness."""

from __future__ import annotations

import datetime
import math
import sys
import uuid

from geometry_msgs.msg import PointStamped
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QApplication, QCheckBox, QComboBox, QDoubleSpinBox, QFormLayout,
    QLabel, QPushButton, QSplitter, QTextEdit, QVBoxLayout, QWidget,
)
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from wvcsc_bringup.qt_image_viewer import RosImagePanel
from wvcsc_interfaces.action import ExecuteSpray
from wvcsc_interfaces.msg import MissionStatus


ARM_TEST_POINT_ID = 'single_arm_test'
OBSERVATION_MODES = {'ik', 'joint_presets'}


def normalize_observation_mode(value):
    mode = str(value).strip().lower()
    if mode not in OBSERVATION_MODES:
        raise ValueError('observation_mode must be ik or joint_presets')
    return mode


def side_distance_coordinates(side, distance_m):
    """Encode a directly-left/right plant hint in the arm-base frame."""
    distance_m = float(distance_m)
    if (side not in {'left', 'right'} or not math.isfinite(distance_m)
            or distance_m <= 0.0):
        raise ValueError('plant side or distance is invalid')
    return 0.0, distance_m if side == 'left' else -distance_m, 0.0


def arm_test_coordinates(
        observation_mode, side, base_distance_m, joint_preset_hint_distance_m):
    """Build the minimal tree hint required by the selected observation mode."""
    mode = normalize_observation_mode(observation_mode)
    distance_m = (
        float(base_distance_m) if mode == 'ik'
        else float(joint_preset_hint_distance_m))
    return side_distance_coordinates(side, distance_m)


def build_spray_goal(
        mission_id, frame_id, x_m, y_m, z_m, duration,
        observation_mode='', working_range_m=0.0):
    """Build the one and only target accepted by the arm-only test UI."""
    goal = ExecuteSpray.Goal()
    goal.mission_id = str(mission_id)
    goal.spray_duration = float(duration)
    goal.tree_hint = PointStamped()
    goal.tree_hint.header.frame_id = str(frame_id)
    goal.tree_hint.point.x = float(x_m)
    goal.tree_hint.point.y = float(y_m)
    goal.tree_hint.point.z = float(z_m)
    goal.observation_mode = str(observation_mode)
    goal.working_range_m = float(working_range_m)
    return goal


class ArmSprayTestNode(Node):
    def __init__(self):
        super().__init__('wvcsc_arm_spray_test_qt')
        self.declare_parameter('base_frame', 'alicia_base_link')
        self.declare_parameter('default_observation_mode', 'joint_presets')
        self.declare_parameter('tree_distance_min_m', 0.80)
        self.declare_parameter('tree_distance_max_m', 1.50)
        self.declare_parameter('working_range_min_m', 0.20)
        self.declare_parameter('working_range_max_m', 2.00)
        self.declare_parameter('default_working_range_m', 1.00)
        self.declare_parameter('joint_preset_hint_distance_m', 1.00)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.default_observation_mode = normalize_observation_mode(
            self.get_parameter('default_observation_mode').value)
        self.tree_distance_min = float(
            self.get_parameter('tree_distance_min_m').value)
        self.tree_distance_max = float(
            self.get_parameter('tree_distance_max_m').value)
        self.working_range_min = float(
            self.get_parameter('working_range_min_m').value)
        self.working_range_max = float(
            self.get_parameter('working_range_max_m').value)
        self.default_working_range = float(
            self.get_parameter('default_working_range_m').value)
        self.joint_preset_hint_distance = float(
            self.get_parameter('joint_preset_hint_distance_m').value)
        if (not math.isfinite(self.tree_distance_min) or
                not math.isfinite(self.tree_distance_max) or
                self.tree_distance_min <= 0.0 or
                self.tree_distance_min > self.tree_distance_max):
            raise ValueError('tree distance range is invalid')
        if (not math.isfinite(self.working_range_min) or
                not math.isfinite(self.working_range_max) or
                self.working_range_min <= 0.0 or
                self.working_range_min > self.working_range_max or
                not self.working_range_min <= self.default_working_range <=
                self.working_range_max):
            raise ValueError('working range is invalid')
        if (not math.isfinite(self.joint_preset_hint_distance) or
                self.joint_preset_hint_distance <= 0.0):
            raise ValueError('joint preset hint distance is invalid')
        self.client = ActionClient(self, ExecuteSpray, '/arm/execute_spray')
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.status_pub = self.create_publisher(
            MissionStatus, '/mission/status', latched)
        self.motion_pub = self.create_publisher(
            String, '/motion_control/command', 10)
        self.motion_state = 'UNKNOWN'
        self.create_subscription(
            String, '/motion_control/state', self._on_motion_state, latched)
        self.goal_handle = None
        self.mission_id = ''
        self.point_id = ''
        self._active = False
        self.on_feedback = None
        self.on_result = None
        self.create_timer(0.5, self._publish_active_status)

    @property
    def active(self):
        return self._active

    def _on_motion_state(self, message):
        self.motion_state = str(message.data)

    def start(
            self, side, base_distance_m, working_range_m, duration,
            observation_mode):
        if self._active:
            raise RuntimeError('spray Action is already active')
        if self.motion_state != 'RUNNING':
            raise RuntimeError(
                f'arm is not ready ({self.motion_state}); wait for HOME to complete')
        if not self.client.wait_for_server(timeout_sec=1.0):
            raise RuntimeError('/arm/execute_spray is unavailable')
        observation_mode = normalize_observation_mode(observation_mode)
        base_distance_m = (
            float(base_distance_m) if observation_mode == 'ik'
            else self.joint_preset_hint_distance)
        if (observation_mode == 'ik' and not
                self.tree_distance_min <= base_distance_m <=
                self.tree_distance_max):
            raise ValueError(
                f'IK plant distance must be within '
                f'{self.tree_distance_min:.2f}-{self.tree_distance_max:.2f} m')
        working_range_m = float(working_range_m)
        if (not math.isfinite(working_range_m) or not
                self.working_range_min <= working_range_m <=
                self.working_range_max):
            raise ValueError(
                f'working_range_m must be within '
                f'{self.working_range_min:.2f}-{self.working_range_max:.2f} m')
        x_m, y_m, z_m = arm_test_coordinates(
            observation_mode, side, base_distance_m,
            self.joint_preset_hint_distance)
        self.mission_id = f'arm_qt_{uuid.uuid4().hex[:8]}'
        self.point_id = ARM_TEST_POINT_ID
        goal = build_spray_goal(
            self.mission_id, self.base_frame,
            x_m, y_m, z_m, duration, observation_mode, working_range_m)
        goal.tree_hint.header.stamp = self.get_clock().now().to_msg()
        self._active = True
        self._publish_active_status()
        future = self.client.send_goal_async(goal, feedback_callback=self._feedback)
        future.add_done_callback(self._goal_response)
        return goal

    def cancel(self):
        if self.goal_handle is None:
            return False
        self.goal_handle.cancel_goal_async()
        return True

    def stop_and_home(self):
        self.motion_pub.publish(String(data='stop'))
        self.cancel()

    def request_home(self):
        self.motion_pub.publish(String(data='reset'))

    def _publish_active_status(self):
        if not self._active:
            return
        message = MissionStatus()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.base_frame
        message.mission_id = self.mission_id
        message.state = MissionStatus.ARM_SPRAYING
        message.state_text = 'ARM_SPRAYING'
        message.current_point_id = self.point_id
        message.current_index = 0
        message.total_points = 1
        message.arm_goal_active = True
        self.status_pub.publish(message)

    def _goal_response(self, future):
        try:
            handle = future.result()
        except Exception as error:
            self._finish(False, ExecuteSpray.Result.INTERNAL_ERROR,
                         f'spray Action request failed: {error}')
            return
        if handle is None or not handle.accepted:
            self._finish(False, ExecuteSpray.Result.BUSY, 'spray Action rejected goal')
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(self._result)

    def _feedback(self, feedback_message):
        feedback = feedback_message.feedback
        if self.on_feedback is not None:
            self.on_feedback(
                int(feedback.phase), float(feedback.progress),
                str(feedback.phase_text))

    def _result(self, future):
        try:
            wrapped = future.result()
            result = wrapped.result
        except Exception as error:
            self._finish(False, ExecuteSpray.Result.INTERNAL_ERROR,
                         f'spray Action result failed: {error}')
            return
        self._finish(bool(result.success), int(result.error_code), str(result.message))

    def _finish(self, success, error_code, message):
        self._active = False
        self.goal_handle = None
        status = MissionStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.header.frame_id = self.base_frame
        status.mission_id = self.mission_id
        canceled = error_code == ExecuteSpray.Result.CANCELED
        status.state = (
            MissionStatus.MISSION_COMPLETED if success else
            (MissionStatus.CANCELED if canceled else MissionStatus.FAILED))
        status.state_text = (
            'MISSION_COMPLETED' if success else
            ('CANCELED' if canceled else 'FAILED'))
        status.total_points = 1
        status.completed_points = 1 if success else 0
        status.last_error = '' if success else message
        self.status_pub.publish(status)
        if self.on_result is not None:
            self.on_result(success, error_code, message)


class ArmSprayTestGui(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.home_pending = False
        self._build_ui()
        self.node.on_feedback = self._feedback
        self.node.on_result = self._result
        self.ros_timer = QTimer(self)
        self.ros_timer.timeout.connect(self._spin_ros)
        self.ros_timer.start(25)
        self._refresh()

    def _build_ui(self):
        self.setWindowTitle('WVCSC 单臂喷洒测试')
        self.setGeometry(260, 170, 1180, 680)
        outer = QVBoxLayout(self)
        controls = QWidget()
        layout = QVBoxLayout(controls)
        form = QFormLayout()
        self.mode_combo = QComboBox()
        self.mode_combo.addItem('IK', 'ik')
        self.mode_combo.addItem('joint_preset', 'joint_presets')
        self.mode_combo.setCurrentIndex(
            max(0, self.mode_combo.findData(self.node.default_observation_mode)))
        form.addRow('观察模式:', self.mode_combo)
        self.duration_spin = self._spin(0.2, 10.0, 5.0)
        form.addRow('喷洒时长 (s):', self.duration_spin)
        self.side_combo = QComboBox()
        self.side_combo.addItem('左侧 (+Y)', 'left')
        self.side_combo.addItem('右侧 (-Y)', 'right')
        form.addRow('病株侧位:', self.side_combo)
        self.base_distance_label = QLabel('基座到病株距离 (m):')
        self.base_distance_spin = self._spin(
            self.node.tree_distance_min, self.node.tree_distance_max, 1.50)
        form.addRow(self.base_distance_label, self.base_distance_spin)
        self.working_range_spin = self._spin(
            self.node.working_range_min, self.node.working_range_max,
            self.node.default_working_range)
        self.working_range_spin.setToolTip(
            '喷嘴轴线投影到目标平面所使用的标定距离；'
            '不替代碰撞、限位或奇异性检查。')
        form.addRow('工作距离 (m):', self.working_range_spin)
        layout.addLayout(form)

        buttons = QVBoxLayout()
        self.start_button = QPushButton('启动')
        self.stop_home_button = QPushButton('复位')
        buttons.addWidget(self.start_button)
        buttons.addWidget(self.stop_home_button)
        layout.addLayout(buttons)
        self.action_label = QLabel('Action: 空闲')
        self.progress_label = QLabel('进度: 0%')
        self.result_label = QLabel('结果: 等待执行')
        self.motion_label = QLabel('机械臂: 等待 /motion_control/state')
        for label in (self.action_label, self.progress_label,
                      self.result_label, self.motion_label):
            layout.addWidget(label)
        self.log_area = QTextEdit()
        self.log_area.setReadOnly(True)
        layout.addWidget(self.log_area, 1)
        self.image_toggle = QCheckBox('显示相机/YOLO画面')
        self.image_toggle.setChecked(True)
        layout.insertWidget(2, self.image_toggle)

        self.image_panel = RosImagePanel(self.node, active=True)
        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(controls)
        splitter.addWidget(self.image_panel)
        splitter.setSizes([430, 750])
        self.output_splitter = splitter
        outer.addWidget(splitter)

        self.start_button.clicked.connect(self._execute)
        self.stop_home_button.clicked.connect(self._stop_and_home)
        self.mode_combo.currentIndexChanged.connect(self._on_mode_changed)
        self.image_toggle.toggled.connect(self._set_image_panel_visible)
        self._on_mode_changed()

    @staticmethod
    def _spin(minimum, maximum, value):
        spin = QDoubleSpinBox()
        spin.setRange(minimum, maximum)
        spin.setDecimals(3)
        spin.setSingleStep(0.05)
        spin.setValue(value)
        return spin

    def _execute(self):
        try:
            observation_mode = str(self.mode_combo.currentData())
            goal = self.node.start(
                str(self.side_combo.currentData()),
                self.base_distance_spin.value(),
                self.working_range_spin.value(),
                self.duration_spin.value(),
                observation_mode)
        except (RuntimeError, ValueError) as error:
            self._log(str(error))
            return
        self.action_label.setText('Action: 请求已发送')
        self.result_label.setText('结果: 等待')
        side_text = '左侧' if goal.tree_hint.point.y > 0.0 else '右侧'
        distance_text = (
            f', 基座距离={abs(goal.tree_hint.point.y):.2f}m'
            if goal.observation_mode == 'ik' else '')
        self._log(
            f'已启动 {goal.observation_mode}: {side_text}{distance_text}, '
            f'工作距离={goal.working_range_m:.2f}m, '
            f'喷洒={goal.spray_duration:.1f}s')

    def _stop_and_home(self):
        self.home_pending = True
        self.node.stop_and_home()
        self._log('已停止喷洒并取消轨迹；正在请求机械臂回 HOME')
        QTimer.singleShot(600, self._request_home)

    def _request_home(self):
        if self.home_pending:
            self.node.request_home()
            self.home_pending = False

    def _on_mode_changed(self, *_args):
        mode = str(self.mode_combo.currentData())
        visible = mode == 'ik'
        self.base_distance_label.setVisible(visible)
        self.base_distance_spin.setVisible(visible)

    def _set_image_panel_visible(self, visible):
        self.image_panel.setVisible(bool(visible))
        self.image_panel.set_active(bool(visible))
        if visible:
            self.output_splitter.setSizes([430, 750])

    def _feedback(self, phase, progress, text):
        self.action_label.setText(f'Action 阶段: {phase} {text}')
        self.progress_label.setText(f'进度: {progress * 100.0:.0f}%')

    def _result(self, success, error_code, message):
        state = '成功' if success else '失败'
        self.result_label.setText(f'结果: {state} (code={error_code}) {message}')
        self._log(self.result_label.text())

    def _spin_ros(self):
        try:
            rclpy.spin_once(self.node, timeout_sec=0.0)
        except RuntimeError:
            return
        self._refresh()

    def _refresh(self):
        self.motion_label.setText(f'机械臂: {self.node.motion_state}')
        self.start_button.setEnabled(
            not self.node.active and not self.home_pending and
            self.node.motion_state == 'RUNNING')

    def _log(self, text):
        timestamp = datetime.datetime.now().strftime('%H:%M:%S')
        self.log_area.append(f'[{timestamp}] {text}')

    def closeEvent(self, event):
        self.ros_timer.stop()
        super().closeEvent(event)


def main(args=None):
    rclpy.init(args=args)
    node = ArmSprayTestNode()
    app = QApplication(sys.argv)
    gui = ArmSprayTestGui(node)
    gui.show()
    try:
        app.exec()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
