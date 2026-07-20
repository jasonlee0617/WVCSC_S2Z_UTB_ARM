# replay_uav_gateway.py
# ============================================================================
# 无人机任务回放网关节点 (Replay UAV Gateway)
# ============================================================================
#
# 职责：
# 1. 按时间轴顺序逐条回放配置文件中定义的多个任务事件。
# 2. 支持 `playback_rate` (倍速) 和 `loop` (循环) 功能。
# 3. 在与无人机真实录像同步播放时，可用于模拟视觉检测结果。
#

import rclpy
from rclpy.node import Node

from .message_factory import mission_message, mission_publisher
from .validation import load_and_validate_replay


class ReplayUavGateway(Node):
    def __init__(self, **kwargs):
        super().__init__('replay_uav_gateway', **kwargs)
        self.declare_parameter('config_file', '')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('min_spray_duration', 0.2)
        self.declare_parameter('max_spray_duration', 10.0)
        self.declare_parameter('max_abs_coordinate', 50.0)

        # 读取并校验 YAML
        self._config = load_and_validate_replay(
            str(self.get_parameter('config_file').value),
            float(self.get_parameter('confidence_threshold').value),
            float(self.get_parameter('min_spray_duration').value),
            float(self.get_parameter('max_spray_duration').value),
            float(self.get_parameter('max_abs_coordinate').value),
        )
        self._publisher = mission_publisher(self)
        self._index = 0
        self._started_at = self._now()

        # 20Hz (0.02s) 高精度轮询定时器
        self._timer = self.create_timer(0.02, self._tick)

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _tick(self):
        """每次定时器触发时，检查是否达到了事件触发的时间。"""
        # 计算经过的仿真时间，并乘以播放速度
        elapsed = (
            (self._now() - self._started_at)
            * self._config['playback_rate'])
        events = self._config['events']

        # 如果当前事件的时间戳 `at_sec` 小于等于已流逝时间，说明可以发布了。
        while self._index < len(events) and events[self._index]['at_sec'] <= elapsed:
            event = events[self._index]
            message = mission_message(
                event['mission'], self.get_clock().now().to_msg())
            self._publisher.publish(message)
            self.get_logger().info(
                f'[UAV_REPLAY] published mission={message.mission_id} '
                f'targets={len(message.trees)} event={self._index + 1}/{len(events)}')
            self._index += 1

        # 如果所有事件都已发布，结束或进入循环逻辑。
        if self._index < len(events):
            return
        if not self._config['loop']:
            self._timer.cancel()
            return
        # 如果开启了循环模式，重置索引并修改起始时间 (加上循环延迟)。
        self._index = 0
        self._started_at = (
            self._now()
            + self._config['loop_delay_sec'] / self._config['playback_rate'])


def main():
    rclpy.init()
    node = ReplayUavGateway()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()