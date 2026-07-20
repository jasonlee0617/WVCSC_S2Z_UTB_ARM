# mock_uav_gateway.py
# ============================================================================
# 模拟无人机任务网关节点 (Mock UAV Gateway)
# ============================================================================
#
# 职责：
# 1. 替代物理无人机，读取硬编码的 YAML 坐标数据。
# 2. 延迟指定时间后，在 ROS 网络上一性次发布完整的病树任务。
# 3. 用于早期逻辑验证、Gazebo 仿真闭环测试。
#

import rclpy
from rclpy.node import Node

from .message_factory import mission_message, mission_publisher
from .validation import load_and_validate


class MockUavGateway(Node):
    def __init__(self, **kwargs):
        super().__init__('mock_uav_gateway', **kwargs)
        self.declare_parameter('config_file', '')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('min_spray_duration', 0.2)
        self.declare_parameter('max_spray_duration', 10.0)
        self.declare_parameter('max_abs_coordinate', 50.0)

        # 读取 YAML 并执行严格的校验
        config = load_and_validate(
            str(self.get_parameter('config_file').value),
            float(self.get_parameter('confidence_threshold').value),
            float(self.get_parameter('min_spray_duration').value),
            float(self.get_parameter('max_spray_duration').value),
            float(self.get_parameter('max_abs_coordinate').value),
        )
        self._config = config
        self._publisher = mission_publisher(self)

        # 创建一个单次触发定时器，等待 `publish_delay_sec` 后触发 `_publish_once`
        self._timer = self.create_timer(
            max(0.001, config['publish_delay_sec']), self._publish_once)

    def _publish_once(self):
        """定时器回调：发布一次任务后取消定时器。"""
        self._timer.cancel()
        message = mission_message(
            self._config, self.get_clock().now().to_msg())
        self._publisher.publish(message)
        self.get_logger().info(
            f"[UAV_GATEWAY] published mission={message.mission_id} "
            f"targets={len(message.trees)} frame={message.header.frame_id}")


def main():
    rclpy.init()
    node = MockUavGateway()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()