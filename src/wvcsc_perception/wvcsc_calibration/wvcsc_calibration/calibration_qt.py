#!/usr/bin/env python3
"""Shared Qt front end for real and Gazebo C10 hand-eye calibration."""

from __future__ import annotations

import datetime
import sys
import threading

from PyQt5.QtCore import QObject, Qt, pyqtSignal
from PyQt5.QtWidgets import (
    QApplication, QCheckBox, QHBoxLayout, QLabel, QMessageBox, QPushButton,
    QSplitter, QTextEdit, QVBoxLayout, QWidget,
)
import rclpy
from rcl_interfaces.msg import Log
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from wvcsc_bringup.qt_image_viewer import RosImagePanel

from .auto_calibration_collector import AutoCalibrationCollector


class _Signals(QObject):
    log = pyqtSignal(str)
    calibration_state = pyqtSignal(str)
    motion_state = pyqtSignal(str)
    request_result = pyqtSignal(str, bool, str)
    reset_finished = pyqtSignal(bool, str)


class CalibrationQtNode(Node):
    """Keep GUI ROS traffic separate from the embedded collector node."""

    def __init__(self, signals):
        super().__init__('calibration_qt')
        self._signals = signals
        self._reset_pending = False
        self._reset_sent = False
        self.latest_calibration_state = 'IDLE'
        self.latest_motion_state = 'UNKNOWN'
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._motion_publisher = self.create_publisher(
            String, '/motion_control/command', 10)
        self.create_subscription(
            String, '/motion_control/state', self._on_motion_state, latched)
        self.create_subscription(
            String, '/calibration/state', self._on_calibration_state, latched)
        self.create_subscription(Log, '/rosout', self._on_log, 100)
        self._calibration_clients = {
            'prepare': self.create_client(Trigger, '/calibration/prepare'),
            'collect': self.create_client(Trigger, '/calibration/collect'),
        }

    def _on_log(self, message):
        if str(message.name).rstrip('/').endswith('auto_calibration_collector'):
            self._signals.log.emit(str(message.msg))

    def _on_calibration_state(self, message):
        state = str(message.data).strip().upper()
        self.latest_calibration_state = state
        self._signals.calibration_state.emit(state)

    def _on_motion_state(self, message):
        state = str(message.data).strip().upper()
        self.latest_motion_state = state
        self._signals.motion_state.emit(state)
        if not self._reset_pending:
            return
        if state == 'STOPPED_LOCKED' and not self._reset_sent:
            self._motion_publisher.publish(String(data='reset'))
            self._reset_sent = True
            self._signals.log.emit('动作已停止，正在执行 HOME')
        elif state == 'RUNNING' and self._reset_sent:
            self._reset_pending = False
            self._reset_sent = False
            self._signals.reset_finished.emit(True, 'HOME 完成，机械臂已就绪')
        elif state == 'RESET_FAILED':
            self._reset_pending = False
            self._reset_sent = False
            self._signals.reset_finished.emit(False, 'HOME 失败，机械臂仍被锁定')

    def request_calibration(self, action):
        client = self._calibration_clients[action]
        if not client.service_is_ready():
            self._signals.request_result.emit(
                action, False, f'/calibration/{action} 服务未就绪')
            return
        future = client.call_async(Trigger.Request())
        future.add_done_callback(
            lambda completed: self._on_request_result(action, completed))

    def _on_request_result(self, action, future):
        try:
            response = future.result()
            self._signals.request_result.emit(
                action, bool(response.success), str(response.message))
        except Exception as error:
            self._signals.request_result.emit(action, False, str(error))

    def stop_and_home(self):
        self._reset_pending = True
        self._reset_sent = False
        self._motion_publisher.publish(String(data='stop'))
        self._signals.log.emit('已请求停止动作；等待 STOPPED_LOCKED 后执行 HOME')


class CalibrationQt(QWidget):
    def __init__(self, node, collector, signals):
        super().__init__()
        self._node = node
        self._collector = collector
        self._signals = signals
        self._calibration_state = 'IDLE'
        self._motion_state = 'UNKNOWN'
        self._arm_moved = False
        self._build_ui()
        signals.log.connect(self._append_log)
        signals.calibration_state.connect(self._set_calibration_state)
        signals.motion_state.connect(self._set_motion_state)
        signals.request_result.connect(self._request_result)
        signals.reset_finished.connect(self._reset_finished)
        self._set_calibration_state(self._node.latest_calibration_state)
        self._set_motion_state(self._node.latest_motion_state)

    def _build_ui(self):
        self.setWindowTitle('WVCSC C10 手眼自动标定')
        self.setMinimumSize(1080, 660)
        self.resize(1240, 740)
        self.setStyleSheet(
            'QWidget { font-size: 14px; } '
            'QPushButton { min-height: 36px; font-weight: 600; } '
            'QTextEdit { background: #1f2329; color: #d8dee9; '
            'font-family: Monospace; font-size: 12px; }')
        outer = QVBoxLayout(self)
        controls = QWidget()
        layout = QVBoxLayout(controls)
        title = QLabel('C10 眼在手自动标定')
        title.setStyleSheet('font-size: 21px; font-weight: 700;')
        layout.addWidget(title)
        self.calibration_label = QLabel('标定状态: IDLE')
        self.motion_label = QLabel('机械臂状态: 等待 /motion_control/state')
        for label in (self.calibration_label, self.motion_label):
            layout.addWidget(label)

        buttons = QHBoxLayout()
        self.start_button = QPushButton('启动：执行标定初始位姿')
        self.collect_button = QPushButton('采集：初始位姿开始标定')
        self.reset_button = QPushButton('复位：停止并执行 HOME')
        buttons.addWidget(self.start_button)
        buttons.addWidget(self.collect_button)
        buttons.addWidget(self.reset_button)
        layout.addLayout(buttons)
        self.image_toggle = QCheckBox('显示相机与标定码可视化')
        self.image_toggle.setChecked(True)
        layout.addWidget(self.image_toggle)
        terminal_title = QLabel('采集器终端输出')
        terminal_title.setStyleSheet('font-weight: 600;')
        layout.addWidget(terminal_title)
        self.log_area = QTextEdit()
        self.log_area.setReadOnly(True)
        layout.addWidget(self.log_area, 1)

        self.image_panel = RosImagePanel(
            self._node, active=True,
            preferred_topics=(
                '/calibration/aruco_debug_image', '/camera/color/image_raw'),
            topic_label='标定图像话题:')
        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(controls)
        splitter.addWidget(self.image_panel)
        splitter.setSizes([450, 790])
        self._splitter = splitter
        outer.addWidget(splitter)

        self.start_button.clicked.connect(
            lambda: self._node.request_calibration('prepare'))
        self.collect_button.clicked.connect(
            lambda: self._node.request_calibration('collect'))
        self.reset_button.clicked.connect(self._reset)
        self.image_toggle.toggled.connect(self._set_image_visible)

    def _append_log(self, message):
        timestamp = datetime.datetime.now().strftime('%H:%M:%S')
        self.log_area.append(f'[{timestamp}] {message}')

    def _set_calibration_state(self, state):
        self._calibration_state = state
        self.calibration_label.setText(f'标定状态: {state}')
        self._refresh()

    def _set_motion_state(self, state):
        self._motion_state = state
        self.motion_label.setText(f'机械臂状态: {state}')
        self._refresh()

    def _request_result(self, action, success, message):
        label = '启动' if action == 'prepare' else '采集'
        self._append_log(f'{label}: {message}')
        if success and action == 'prepare':
            self._arm_moved = True
        self._refresh()

    def _reset(self):
        self.reset_button.setEnabled(False)
        self._node.stop_and_home()

    def _reset_finished(self, success, message):
        self._append_log(message)
        if success:
            self._arm_moved = False
            self._collector.mark_home()
        self._refresh()

    def _set_image_visible(self, visible):
        self.image_panel.setVisible(bool(visible))
        self.image_panel.set_active(bool(visible))
        self._splitter.setSizes([450, 790] if visible else [1240, 0])

    def _refresh(self):
        ready = self._motion_state == 'RUNNING'
        self.start_button.setEnabled(
            ready and self._calibration_state in {'IDLE', 'READY'})
        self.collect_button.setEnabled(
            ready and self._calibration_state == 'READY')
        self.reset_button.setEnabled(self._motion_state != 'RESETTING')

    def closeEvent(self, event):
        if self._arm_moved or self._collector.moved_from_home:
            QMessageBox.warning(
                self, '机械臂尚未 HOME',
                '机械臂已离开 HOME。请先点击“复位：停止并执行 HOME”，'
                '确认完成后再关闭界面。')
            event.ignore()
            return
        super().closeEvent(event)


def main(args=None):
    rclpy.init(args=args)
    signals = _Signals()
    collector = AutoCalibrationCollector(enable_keyboard=False)
    node = CalibrationQtNode(signals)
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(collector)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    app = QApplication(sys.argv)
    gui = CalibrationQt(node, collector, signals)
    gui.show()
    try:
        app.exec()
    finally:
        executor.shutdown()
        node.destroy_node()
        collector.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
