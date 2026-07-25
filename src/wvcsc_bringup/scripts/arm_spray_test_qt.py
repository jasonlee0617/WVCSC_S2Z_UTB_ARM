#!/usr/bin/env python3
"""Single-target Alicia-M spray test with operator-visible recovery controls."""

from __future__ import annotations

import datetime
import sys
import uuid

from geometry_msgs.msg import PointStamped
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import (
    QApplication, QDoubleSpinBox, QFormLayout, QGridLayout, QHBoxLayout,
    QLabel, QLineEdit, QPushButton, QTextEdit, QVBoxLayout, QWidget,
)
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from wvcsc_bringup.qt_image_viewer import RosImagePanel
from wvcsc_interfaces.action import ExecuteSpray
from wvcsc_interfaces.msg import MissionStatus


def build_spray_goal(mission_id, tree_id, frame_id, x_m, y_m, z_m, duration):
    """Build the one and only target accepted by the arm-only test UI."""
    goal = ExecuteSpray.Goal()
    goal.mission_id = str(mission_id)
    goal.tree_id = str(tree_id)
    goal.spray_duration = float(duration)
    goal.tree_hint = PointStamped()
    goal.tree_hint.header.frame_id = str(frame_id)
    goal.tree_hint.point.x = float(x_m)
    goal.tree_hint.point.y = float(y_m)
    goal.tree_hint.point.z = float(z_m)
    return goal


class ArmSprayTestNode(Node):
    def __init__(self):
        super().__init__('wvcsc_arm_spray_test_qt')
        self.declare_parameter('base_frame', 'alicia_base_link')
        self.base_frame = str(self.get_parameter('base_frame').value)
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
        self.target_id = ''
        self._active = False
        self.on_feedback = None
        self.on_result = None
        self.create_timer(0.5, self._publish_active_status)

    @property
    def active(self):
        return self._active

    def _on_motion_state(self, message):
        self.motion_state = str(message.data)

    def start(self, tree_id, x_m, y_m, z_m, duration):
        if self._active:
            raise RuntimeError('spray Action is already active')
        if not self.client.wait_for_server(timeout_sec=1.0):
            raise RuntimeError('/arm/execute_spray is unavailable')
        self.mission_id = f'arm_qt_{uuid.uuid4().hex[:8]}'
        self.target_id = str(tree_id).strip() or 'single_target'
        goal = build_spray_goal(
            self.mission_id, self.target_id, self.base_frame,
            x_m, y_m, z_m, duration)
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

    def unlock_after_home(self):
        if self.motion_state != 'HOME_LOCKED':
            return False
        self.motion_pub.publish(String(data='resume'))
        return True

    def _publish_active_status(self):
        if not self._active:
            return
        message = MissionStatus()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.base_frame
        message.mission_id = self.mission_id
        message.state = MissionStatus.ARM_SPRAYING
        message.state_text = 'ARM_SPRAYING'
        message.current_tree_id = self.target_id
        message.current_index = 0
        message.total_targets = 1
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
        status.total_targets = 1
        status.completed_targets = 1 if success else 0
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
        outer = QHBoxLayout(self)
        controls = QWidget()
        layout = QVBoxLayout(controls)
        form = QFormLayout()
        self.tree_id = QLineEdit('corn_01')
        form.addRow('树/病株 ID:', self.tree_id)
        self.x_spin = self._spin(-5.0, 5.0, 0.0)
        self.y_spin = self._spin(-5.0, 5.0, 1.50)
        self.z_spin = self._spin(-2.0, 3.0, 0.0)
        self.duration_spin = self._spin(0.2, 10.0, 5.0)
        form.addRow('树 X (m):', self.x_spin)
        form.addRow('树 Y (m):', self.y_spin)
        form.addRow('树 Z (m):', self.z_spin)
        form.addRow('喷洒时长 (s):', self.duration_spin)
        layout.addLayout(form)
        buttons = QGridLayout()
        self.execute_button = QPushButton('执行单目标喷洒')
        self.cancel_button = QPushButton('取消 Action')
        self.stop_home_button = QPushButton('停止并回 HOME')
        self.unlock_button = QPushButton('HOME 完成后解锁')
        buttons.addWidget(self.execute_button, 0, 0, 1, 2)
        buttons.addWidget(self.cancel_button, 1, 0)
        buttons.addWidget(self.stop_home_button, 1, 1)
        buttons.addWidget(self.unlock_button, 2, 0, 1, 2)
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
        outer.addWidget(controls, 2)
        outer.addWidget(RosImagePanel(self.node), 3)

        self.execute_button.clicked.connect(self._execute)
        self.cancel_button.clicked.connect(self._cancel)
        self.stop_home_button.clicked.connect(self._stop_and_home)
        self.unlock_button.clicked.connect(self._unlock)

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
            goal = self.node.start(
                self.tree_id.text(), self.x_spin.value(), self.y_spin.value(),
                self.z_spin.value(), self.duration_spin.value())
        except RuntimeError as error:
            self._log(str(error))
            return
        self.action_label.setText('Action: 请求已发送')
        self.result_label.setText('结果: 等待')
        self._log(
            f'已发送 {goal.tree_id}: '
            f'({goal.tree_hint.point.x:.2f}, {goal.tree_hint.point.y:.2f}, '
            f'{goal.tree_hint.point.z:.2f}) m, {goal.spray_duration:.1f}s')

    def _cancel(self):
        self._log('已请求取消当前喷洒 Action' if self.node.cancel() else '没有可取消的 Action')

    def _stop_and_home(self):
        self.home_pending = True
        self.node.stop_and_home()
        self._log('已停止喷洒并取消轨迹；正在请求机械臂回 HOME')
        QTimer.singleShot(600, self._request_home)

    def _request_home(self):
        if self.home_pending:
            self.node.request_home()
            self.home_pending = False

    def _unlock(self):
        if self.node.unlock_after_home():
            self._log('已发送解除 HOME 锁定请求')
        else:
            self._log('机械臂尚未到达 HOME_LOCKED，不能解锁')

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
        self.execute_button.setEnabled(not self.node.active)
        self.cancel_button.setEnabled(self.node.active)
        self.unlock_button.setEnabled(self.node.motion_state == 'HOME_LOCKED')

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
