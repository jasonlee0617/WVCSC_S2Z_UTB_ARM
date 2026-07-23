#!/usr/bin/env python3
"""ROS 2 继电器服务请求示例。"""

import argparse
import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from wvcsc_interfaces.srv import SetRelay


class RelayClient(Node):
    """调用 /relay/set 服务控制继电器。"""

    def __init__(self):
        super().__init__('relay_client_demo')

        # 创建自定义服务客户端，服务端由 relay_controller 节点提供。
        self._client = self.create_client(SetRelay, '/relay/set')

    def set_relay(
        self,
        channel: int,
        enabled: bool,
        duration: float = 0.0,
    ) -> bool:
        """发送继电器控制请求，并等待服务端返回结果。"""

        # 服务端可能还没有启动，因此先等待服务上线。
        while not self._client.wait_for_service(timeout_sec=1.0):
            if not rclpy.ok():
                return False
            self.get_logger().info('正在等待 /relay/set 服务……')

        # 请求中同时指定通道号和需要设置的状态。
        request = SetRelay.Request()
        request.channel = channel
        request.enabled = enabled
        request.duration = duration

        state = '吸合' if enabled else '断开'
        if enabled and duration > 0.0:
            self.get_logger().info(
                f'请求第 {channel} 路继电器{state} {duration:.3f} 秒'
            )
        else:
            self.get_logger().info(f'请求第 {channel} 路继电器{state}')

        # 异步发送请求，再运行节点直到收到响应。
        future = self._client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.exception() is not None:
            self.get_logger().error(f'服务调用异常：{future.exception()}')
            return False

        response = future.result()
        if response is None:
            self.get_logger().error('没有收到服务响应')
            return False

        if response.success:
            self.get_logger().info(f'控制成功：{response.message}')
        else:
            self.get_logger().error(f'控制失败：{response.message}')
        return response.success


def main(args=None):
    rclpy.init(args=args)

    # 去除 --ros-args 等 ROS 参数，只解析 Demo 自己的 on/off 参数。
    cli_args = remove_ros_args(args=sys.argv if args is None else args)
    parser = argparse.ArgumentParser(description='请求 ROS 2 服务控制继电器')
    parser.add_argument(
        'channel',
        type=int,
        help='继电器通道号，范围为 1～255',
    )
    parser.add_argument(
        'state',
        choices=('on', 'off'),
        help='on=继电器吸合，off=继电器断开',
    )
    parser.add_argument(
        'duration',
        type=float,
        nargs='?',
        default=0.0,
        help='吸合持续秒数，默认 0（持续吸合）',
    )
    parsed_args = parser.parse_args(cli_args[1:])
    if not 1 <= parsed_args.channel <= 255:
        parser.error('channel 必须在 1～255 范围内')
    if not math.isfinite(parsed_args.duration) or parsed_args.duration < 0.0:
        parser.error('duration 必须是大于等于 0 的有限数值')

    node = RelayClient()
    try:
        success = node.set_relay(
            parsed_args.channel,
            parsed_args.state == 'on',
            parsed_args.duration,
        )
    except KeyboardInterrupt:
        success = False
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    return 0 if success else 1


if __name__ == '__main__':
    raise SystemExit(main())
