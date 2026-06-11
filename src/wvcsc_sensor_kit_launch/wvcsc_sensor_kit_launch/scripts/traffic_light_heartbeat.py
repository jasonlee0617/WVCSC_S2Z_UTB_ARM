#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from autoware_perception_msgs.msg import TrafficLightGroupArray


class TrafficLightHeartbeat(Node):
    def __init__(self) -> None:
        super().__init__("traffic_light_heartbeat")
        topic = self.declare_parameter(
            "output_topic", "/perception/traffic_light_recognition/traffic_signals"
        ).value
        rate_hz = float(self.declare_parameter("rate_hz", 5.0).value)

        self._publisher = self.create_publisher(TrafficLightGroupArray, topic, 1)
        self._timer = self.create_timer(1.0 / max(rate_hz, 0.1), self._on_timer)
        self.get_logger().info(
            f"Publishing empty traffic light heartbeat on {topic} at {rate_hz:.1f} Hz"
        )

    def _on_timer(self) -> None:
        msg = TrafficLightGroupArray()
        msg.stamp = self.get_clock().now().to_msg()
        self._publisher.publish(msg)


def main() -> None:
    rclpy.init()
    node = TrafficLightHeartbeat()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
