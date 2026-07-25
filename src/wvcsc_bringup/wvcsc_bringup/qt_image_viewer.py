"""Small reusable raw ROS image panel for the WVCSC Qt tools."""

from __future__ import annotations

from cv_bridge import CvBridge
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import QComboBox, QHBoxLayout, QLabel, QVBoxLayout, QWidget
from rclpy.qos import qos_profile_sensor_data


IMAGE_TYPE = 'sensor_msgs/msg/Image'
PREFERRED_IMAGE_TOPICS = (
    '/camera/color/image_raw',
    '/vision/tree_debug_image',
    '/vision/diseased_target_debug_image',
)


def image_topic_names(topic_types):
    """Return raw Image topics, with the field-camera topics first."""
    names = {
        name for name, types in topic_types
        if IMAGE_TYPE in types
    }
    preferred_order = {name: index for index, name in enumerate(PREFERRED_IMAGE_TOPICS)}
    return sorted(names, key=lambda name: (preferred_order.get(name, 999), name))


class RosImagePanel(QWidget):
    """Discover and render ``sensor_msgs/Image`` topics from one ROS node."""

    def __init__(self, node, parent=None):
        super().__init__(parent)
        self._node = node
        self._bridge = CvBridge()
        self._subscription = None
        self._topic = ''

        layout = QVBoxLayout(self)
        controls = QHBoxLayout()
        controls.addWidget(QLabel('YOLO/相机图像话题:'))
        self.topic_combo = QComboBox()
        self.topic_combo.addItem('等待 sensor_msgs/Image 话题', '')
        controls.addWidget(self.topic_combo, 1)
        layout.addLayout(controls)
        self.image_label = QLabel('等待图像')
        self.image_label.setAlignment(Qt.AlignCenter)
        self.image_label.setMinimumSize(480, 320)
        self.image_label.setStyleSheet('background: #202020; color: #dddddd;')
        layout.addWidget(self.image_label, 1)

        self.topic_combo.currentIndexChanged.connect(self._select_topic)
        self._discovery_timer = QTimer(self)
        self._discovery_timer.timeout.connect(self.refresh_topics)
        self._discovery_timer.start(1000)
        self.refresh_topics()

    def refresh_topics(self):
        try:
            topics = image_topic_names(self._node.get_topic_names_and_types())
        except Exception:
            return
        current = self._topic
        existing = [self.topic_combo.itemData(index)
                    for index in range(self.topic_combo.count())]
        if topics == existing[1:]:
            return
        self.topic_combo.blockSignals(True)
        self.topic_combo.clear()
        if not topics:
            self.topic_combo.addItem('等待 sensor_msgs/Image 话题', '')
        else:
            for topic in topics:
                self.topic_combo.addItem(topic, topic)
            index = self.topic_combo.findData(current)
            self.topic_combo.setCurrentIndex(index if index >= 0 else 0)
        self.topic_combo.blockSignals(False)
        self._select_topic(self.topic_combo.currentIndex())

    def _select_topic(self, _index):
        topic = self.topic_combo.currentData() or ''
        if topic == self._topic:
            return
        if self._subscription is not None:
            self._node.destroy_subscription(self._subscription)
            self._subscription = None
        self._topic = topic
        if not topic:
            self.image_label.setText('等待图像')
            self.image_label.setPixmap(QPixmap())
            return
        from sensor_msgs.msg import Image
        self._subscription = self._node.create_subscription(
            Image, topic, self._on_image, qos_profile_sensor_data)
        self.image_label.setText(f'等待 {topic}')

    def _on_image(self, message):
        try:
            bgr = self._bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
            rgb = bgr[:, :, ::-1].copy()
            image = QImage(
                rgb.data, rgb.shape[1], rgb.shape[0], rgb.strides[0],
                QImage.Format_RGB888).copy()
        except Exception as error:
            self.image_label.setText(f'图像转换失败: {error}')
            return
        pixmap = QPixmap.fromImage(image)
        self.image_label.setPixmap(pixmap.scaled(
            self.image_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))

    def closeEvent(self, event):
        self._discovery_timer.stop()
        if self._subscription is not None:
            self._node.destroy_subscription(self._subscription)
            self._subscription = None
        super().closeEvent(event)
