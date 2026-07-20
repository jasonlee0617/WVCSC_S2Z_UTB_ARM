# camera_watchdog.py
# ============================================================================
# C10 相机状态监控看门狗节点 (Camera Watchdog)
# ============================================================================
#
# 职责：
# 作为一个非介入式的"质检员"，它不修改任何图像数据，而是通过持续监听
# ROS 相机话题，实时判断相机的健康状况：
# 1. 图像流是否断开或长时间停止更新。
# 2. 实际测量的帧率是否远低于设定值。
# 3. 图像分辨率是否突然改变（可能是驱动配置错误）。
# 4. 图像时间戳是否发生倒流（可能是硬件驱动 Bug）。
#
# 所有检测结果通过标准的 `/diagnostics` 话题上报，便于上位机监控软件
# （如 `robot_monitor`）或 Web 界面直观地展示相机健康度。
#

from collections import deque

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


class CameraWatchdog(Node):
    def __init__(self):
        super().__init__('wvcsc_c10_watchdog')

        # 1. 声明并加载所有 ROS2 参数。
        # 注意：这里的默认话题为 `/color/image_raw` 和 `/color/camera_info`，
        # 这表示该看门狗预期相机节点发布在这些话题上。如果您通过 `Launch` 
        # 传入参数，可以覆盖这些默认值。
        defaults = {
            'image_topic': '/color/image_raw',           # 图像话题
            'camera_info_topic': '/color/camera_info',   # 相机内参话题
            'expected_width': 1280,                      # 期望分辨率宽度
            'expected_height': 720,                      # 期望分辨率高度
            'expected_fps': 30.0,                        # 期望帧率
            'stale_timeout_sec': 1.0,                    # 判定流过期的超时时间（秒）
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)

        # 2. 状态缓存变量初始化
        self._last_image_time = None      # 最后一次收到图像时的 ROS 时刻
        self._last_header_stamp = None    # 最后一次收到图像的 Header 时间戳
        self._last_info = None            # 最后一次收到的 CameraInfo 消息
        self._image = None                # 最近一帧的 Image 消息

        # 3. 历史数据记录容器（定长双端队列）
        # `deque(maxlen=120)` 可实现滑动窗口统计，用于平稳地计算实际帧率。
        self._samples = deque(maxlen=120)
        # 记录每一帧的时间戳是否比上一帧更小（时间倒流），用于监测单调性。
        self._stamp_regressions = deque(maxlen=120)

        # 4. 创建诊断消息发布器（发布到 ROS 标准 `/diagnostics` 话题）
        self._publisher = self.create_publisher(
            DiagnosticArray, '/diagnostics', 10)

        # 5. 订阅相机图像和内参话题
        self.create_subscription(
            Image, str(self.get_parameter('image_topic').value),
            self._on_image, qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo, str(self.get_parameter('camera_info_topic').value),
            self._on_info, qos_profile_sensor_data)

        # 6. 创建 1 秒周期的定时器，定期触发诊断评估与发布。
        self.create_timer(1.0, self._publish)

    def _on_image(self, message):
        """图像回调：接收并记录帧到达的时间、时间戳等信息。"""
        now = self.get_clock().now()
        # 提取图像的 Header 时间戳（秒）
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        
        # 检查当前图像的时间戳是否 <= 上一帧的时间戳（代表时间倒流）
        self._stamp_regressions.append(
            self._last_header_stamp is not None
            and stamp <= self._last_header_stamp)
        
        # 更新缓存状态
        self._last_header_stamp = stamp
        self._last_image_time = now
        self._image = message
        # 记录当前的 ROS 时间（纳秒转秒）至队列，用于计算帧率
        self._samples.append(now.nanoseconds * 1e-9)

    def _on_info(self, message):
        """内参回调：记录 CameraInfo 消息。"""
        self._last_info = message

    def _measured_fps(self):
        """根据 `_samples` 滑动窗口计算实际帧率。"""
        if len(self._samples) < 2:
            return 0.0
        span = self._samples[-1] - self._samples[0]  # 窗口首尾的时间跨度
        # 帧率 = (窗口内帧数 - 1) / 时间跨度
        return (len(self._samples) - 1) / span if span > 0.0 else 0.0

    def _publish(self):
        """
        核心诊断评估函数：执行多项规则检查，生成 `DiagnosticStatus` 并发布。
        """
        expected_w = int(self.get_parameter('expected_width').value)
        expected_h = int(self.get_parameter('expected_height').value)
        expected_fps = float(self.get_parameter('expected_fps').value)
        stale = float(self.get_parameter('stale_timeout_sec').value)

        status = DiagnosticStatus()
        status.name = 'wvcsc/c10_camera'
        status.hardware_id = 'Synria-C10'

        now = self.get_clock().now()
        # 计算图像 Header 时间戳与当前 ROS 时钟的时间差
        stamp_age = (
            now.nanoseconds * 1e-9 - self._last_header_stamp
            if self._last_header_stamp is not None else float('inf'))

        # ------------- 诊断逻辑判定 (优先级从高到低) -------------
        
        # 1. 是否从未收到过图像，或距离上次图像收到时间超过 `stale` 阈值？
        if self._last_image_time is None or (
                now - self._last_image_time).nanoseconds * 1e-9 > stale:
            status.level = DiagnosticStatus.ERROR
            status.message = 'image stream missing or stale'

        # 2. 图像分辨率是否与预设不符？
        elif self._image.width != expected_w or self._image.height != expected_h:
            status.level = DiagnosticStatus.ERROR
            status.message = 'unexpected image resolution'

        # 3. 图像编码格式是否合规 (必须为 ROS 常见的 rgb8 或 bgr8)？
        elif self._image.encoding not in ('rgb8', 'bgr8'):
            status.level = DiagnosticStatus.ERROR
            status.message = f'unexpected ROS encoding: {self._image.encoding}'

        # 4. 图像 Header 中的时间戳是否缺失或无效？
        elif self._last_header_stamp is None or self._last_header_stamp <= 0.0:
            status.level = DiagnosticStatus.ERROR
            status.message = 'image timestamp is missing'

        # 5. 图像时间戳是否发生了"倒流"（不单调递增）？
        # 如果 `_stamp_regressions` 队列中有任何 `True` 的记录，则报警。
        elif sum(self._stamp_regressions) > 0:
            status.level = DiagnosticStatus.WARN
            status.message = 'image timestamp is not monotonic'

        # 6. 图像时间戳与当前系统时间差值过大（可能系统时钟发生了跳跃）？
        elif abs(stamp_age) > stale:
            status.level = DiagnosticStatus.WARN
            status.message = 'image timestamp differs from ROS clock'

        # 7. 是否从未收到过 CameraInfo 标定信息？
        elif self._last_info is None:
            status.level = DiagnosticStatus.WARN
            status.message = 'CameraInfo not received'

        # 8. 实际测量的帧率是否低于期望值的 70% (即低于 21 Hz)？
        elif self._measured_fps() < expected_fps * 0.70:
            status.level = DiagnosticStatus.WARN
            status.message = 'camera frame rate below threshold'

        # 9. 所有检查均通过，相机状态健康。
        else:
            status.level = DiagnosticStatus.OK
            status.message = 'camera stream healthy'

        # ------------- 填充诊断 Key-Value 数据 -------------
        width = self._image.width if self._image is not None else 0
        height = self._image.height if self._image is not None else 0
        encoding = self._image.encoding if self._image is not None else ''
        status.values = [
            KeyValue(key='resolution', value=f'{width}x{height}'),
            KeyValue(key='encoding', value=encoding),
            KeyValue(key='measured_fps', value=f'{self._measured_fps():.2f}'),
            KeyValue(key='expected_fps', value=f'{expected_fps:.2f}'),
            KeyValue(key='stamp_age_sec', value=f'{stamp_age:.3f}'),
            KeyValue(
                key='recent_stamp_regressions',
                value=str(sum(self._stamp_regressions))),
        ]

        # ------------- 打包并发布诊断消息 -------------
        message = DiagnosticArray()
        message.header.stamp = now.to_msg()
        message.status = [status]
        self._publisher.publish(message)


def main():
    rclpy.init()
    node = CameraWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()