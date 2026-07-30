"""Small reusable raw ROS image panel for the WVCSC Qt tools."""

from __future__ import annotations

import time

import numpy as np
from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import QComboBox, QHBoxLayout, QLabel, QVBoxLayout, QWidget
from rclpy.qos import qos_profile_sensor_data


IMAGE_TYPE = 'sensor_msgs/msg/Image'
PREFERRED_IMAGE_TOPICS = (
    '/camera/color/image_raw',
    '/vision/diseased_target_debug_image',
)


def image_to_qimage(message):
    """Convert the common raw ROS image encodings without importing OpenCV.

    Importing ``cv_bridge`` loads the user's OpenCV wheel on some field
    computers.  That wheel may redirect Qt to its bundled platform plugin
    before ``QApplication`` starts, which conflicts with PyQt5's ``xcb``
    plugin.  The navigation panel only needs raw 8-bit camera/debug images,
    so convert those bytes directly and keep the Qt process OpenCV-free.
    """
    encoding = str(message.encoding).lower()
    channels_by_encoding = {
        'bgr8': 3,
        'rgb8': 3,
        'bgra8': 4,
        'rgba8': 4,
        'mono8': 1,
    }
    channels = channels_by_encoding.get(encoding)
    if channels is None:
        raise ValueError(f'unsupported image encoding: {message.encoding}')

    height = int(message.height)
    width = int(message.width)
    step = int(message.step)
    if height <= 0 or width <= 0 or step < width * channels:
        raise ValueError('invalid image dimensions or row stride')

    expected_size = height * step
    raw = np.frombuffer(message.data, dtype=np.uint8)
    if raw.size < expected_size:
        raise ValueError('image data is shorter than its declared row stride')
    pixels = raw[:expected_size].reshape(height, step)
    pixels = pixels[:, :width * channels].reshape(height, width, channels)

    if encoding == 'bgr8':
        rgb = pixels[:, :, ::-1].copy()
        return QImage(
            rgb.data, width, height, rgb.strides[0], QImage.Format_RGB888).copy()
    if encoding == 'rgb8':
        rgb = pixels.copy()
        return QImage(
            rgb.data, width, height, rgb.strides[0], QImage.Format_RGB888).copy()
    if encoding == 'bgra8':
        rgba = pixels[:, :, [2, 1, 0, 3]].copy()
        return QImage(
            rgba.data, width, height, rgba.strides[0],
            QImage.Format_RGBA8888).copy()
    if encoding == 'rgba8':
        rgba = pixels.copy()
        return QImage(
            rgba.data, width, height, rgba.strides[0],
            QImage.Format_RGBA8888).copy()

    gray = pixels[:, :, 0].copy()
    return QImage(
        gray.data, width, height, gray.strides[0], QImage.Format_Grayscale8).copy()


def image_topic_names(topic_types, preferred_topics=PREFERRED_IMAGE_TOPICS):
    """Return raw Image topics, with the field-camera topics first."""
    names = {
        name for name, types in topic_types
        if IMAGE_TYPE in types
    }
    preferred_order = {name: index for index, name in enumerate(preferred_topics)}
    return sorted(names, key=lambda name: (preferred_order.get(name, 999), name))


def selected_image_topic(topics, preferred_topics, current_topic='',
                         manual_selection=False):
    """Keep a manual choice, otherwise promote the first available preference."""
    topics = tuple(topics)
    # ROS graph discovery can transiently omit a still-live topic.  Retaining
    # its subscription avoids blanking the panel and discarding its last frame
    # during that one discovery tick.
    if current_topic and current_topic not in topics:
        return current_topic
    if manual_selection and current_topic:
        return current_topic
    if not manual_selection:
        for topic in preferred_topics:
            if topic in topics:
                return topic
    if current_topic:
        return current_topic
    return topics[0] if topics else ''


class RosImagePanel(QWidget):
    """Discover and render ``sensor_msgs/Image`` topics from one ROS node."""

    _image_ready = pyqtSignal(QImage)
    _image_error = pyqtSignal(str)

    def __init__(self, node, parent=None, active=True,
                 preferred_topics=PREFERRED_IMAGE_TOPICS, topic_label='YOLO/相机图像话题:'):
        super().__init__(parent)
        self._node = node
        self._subscription = None
        self._topic = ''
        self._active = False
        self._preferred_topics = tuple(preferred_topics)
        self._manual_selection = False
        self._last_frame_at = None
        self._last_emit_at = 0.0

        layout = QVBoxLayout(self)
        controls = QHBoxLayout()
        controls.addWidget(QLabel(topic_label))
        self.topic_combo = QComboBox()
        self.topic_combo.addItem('等待 sensor_msgs/Image 话题', '')
        controls.addWidget(self.topic_combo, 1)
        layout.addLayout(controls)
        self.stream_status = QLabel('等待图像')
        layout.addWidget(self.stream_status)
        self.image_label = QLabel('等待图像')
        self.image_label.setAlignment(Qt.AlignCenter)
        self.image_label.setMinimumSize(480, 320)
        self.image_label.setStyleSheet('background: #202020; color: #dddddd;')
        layout.addWidget(self.image_label, 1)

        self.topic_combo.currentIndexChanged.connect(self._select_topic)
        self.topic_combo.activated.connect(self._mark_manual_selection)
        self._image_ready.connect(self._display_image, Qt.QueuedConnection)
        self._image_error.connect(self._display_error, Qt.QueuedConnection)
        self._discovery_timer = QTimer(self)
        self._discovery_timer.timeout.connect(self.refresh_topics)
        self._stream_timer = QTimer(self)
        self._stream_timer.timeout.connect(self._refresh_stream_status)
        self.set_active(active)

    def set_active(self, active):
        """Start or stop discovery and subscriptions with the panel visibility."""
        active = bool(active)
        if active == self._active:
            return
        self._active = active
        if active:
            self._discovery_timer.start(1000)
            self._stream_timer.start(250)
            self.refresh_topics(force=True)
            return
        self._discovery_timer.stop()
        self._stream_timer.stop()
        self._destroy_subscription()
        self.stream_status.setText('图像面板已收起')
        self.image_label.setText('图像面板已收起')
        self.image_label.setPixmap(QPixmap())

    def refresh_topics(self, force=False):
        if not self._active:
            return
        try:
            topics = image_topic_names(
                self._node.get_topic_names_and_types(), self._preferred_topics)
        except Exception:
            return
        current = self._topic
        selected = selected_image_topic(
            topics, self._preferred_topics, current, self._manual_selection)
        visible_topics = list(topics)
        if selected and selected not in visible_topics:
            visible_topics.append(selected)
        visible_topics.sort(key=lambda topic: (
            self._preferred_topics.index(topic)
            if topic in self._preferred_topics else 999, topic))
        existing = [self.topic_combo.itemData(index)
                    for index in range(self.topic_combo.count())
                    if self.topic_combo.itemData(index)]
        if not force and visible_topics == existing:
            return
        self.topic_combo.blockSignals(True)
        self.topic_combo.clear()
        if not visible_topics:
            self.topic_combo.addItem('等待 sensor_msgs/Image 话题', '')
        else:
            for topic in visible_topics:
                self.topic_combo.addItem(topic, topic)
            index = self.topic_combo.findData(selected)
            self.topic_combo.setCurrentIndex(index if index >= 0 else 0)
        self.topic_combo.blockSignals(False)
        self._select_topic(self.topic_combo.currentIndex())

    def _mark_manual_selection(self, _index):
        self._manual_selection = True

    def _select_topic(self, _index):
        topic = self.topic_combo.currentData() or ''
        if topic == self._topic and self._subscription is not None:
            return
        self._destroy_subscription()
        self._topic = topic
        self._last_frame_at = None
        if not topic or not self._active:
            self.stream_status.setText('等待图像')
            self.image_label.setText('等待图像')
            self.image_label.setPixmap(QPixmap())
            return
        from sensor_msgs.msg import Image
        self._subscription = self._node.create_subscription(
            Image, topic, self._on_image, qos_profile_sensor_data)
        self.stream_status.setText(f'等待 {topic}')
        self.image_label.setText(f'等待 {topic}')

    def _destroy_subscription(self):
        if self._subscription is not None:
            self._node.destroy_subscription(self._subscription)
            self._subscription = None

    def _on_image(self, message):
        now = time.monotonic()
        if now - self._last_emit_at < 1.0 / 15.0:
            return
        try:
            image = image_to_qimage(message)
        except Exception as error:
            self._image_error.emit(f'图像转换失败: {error}')
            return
        self._last_emit_at = now
        self._image_ready.emit(image)

    def _display_image(self, image):
        self._last_frame_at = time.monotonic()
        self.stream_status.setText('图像正常')
        pixmap = QPixmap.fromImage(image)
        self.image_label.setPixmap(pixmap.scaled(
            self.image_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))

    def _display_error(self, message):
        self.stream_status.setText(message)

    def _refresh_stream_status(self):
        if not self._active or not self._topic:
            return
        if self._last_frame_at is None:
            self.stream_status.setText(f'等待 {self._topic}')
        elif time.monotonic() - self._last_frame_at > 1.0:
            self.stream_status.setText(f'图像流中断，正在重连 {self._topic}')

    def closeEvent(self, event):
        self._discovery_timer.stop()
        self._stream_timer.stop()
        self._destroy_subscription()
        super().closeEvent(event)
