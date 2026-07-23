import math

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from wvcsc_interfaces.srv import SetRelay

from controller_pkg.set_fault import RelayController as ModbusRelayController


class RelayController(Node):
    """将 Modbus RTU 继电器封装为 ROS 2 服务。"""

    def __init__(self):
        super().__init__('relay_controller')

        # 默认读取安装到 share/controller_pkg/config 下的串口配置文件。
        default_config = (
            get_package_share_directory('controller_pkg')
            + '/config/fault.ini'
        )
        self.declare_parameter('config_file', default_config)
        config_file = self.get_parameter('config_file').value

        # 复用 example.py 中使用的 Modbus RTU 控制器。
        # 该对象负责生成 0x05 命令、CRC 校验以及串口收发。
        self._relay = ModbusRelayController(config_file)
        # 记录本节点成功吸合过的通道，退出时用于安全断开。
        self._active_channels = set()
        # 每个通道分别保存自动断开定时器，通道之间互不影响。
        self._off_timers = {}
        self._service = self.create_service(
            SetRelay,
            'relay/set',
            self._set_relay_callback,
        )
        self.get_logger().info('继电器服务已启动：/relay/set')

    def _write_relay(self, channel: int, enabled: bool) -> bool:
        """控制继电器并记录状态；True=吸合，False=断开。"""
        success = self._relay.set_channel(channel, enabled)
        if not success:
            return False

        if enabled:
            self._active_channels.add(channel)
        else:
            self._active_channels.discard(channel)
        return True

    def _cancel_off_timer(self, channel: int) -> None:
        """取消并移除指定通道原有的自动断开定时器。"""
        timer = self._off_timers.pop(channel, None)
        if timer is not None:
            timer.cancel()
            self.destroy_timer(timer)

    def _schedule_auto_off(self, channel: int, duration: float) -> None:
        """创建一次性定时任务，到期后自动断开指定通道。"""
        self._cancel_off_timer(channel)

        def auto_off_callback():
            # ROS 2 Timer 默认会周期执行，进入回调后先取消，实现一次性效果。
            self._cancel_off_timer(channel)
            try:
                if self._write_relay(channel, False):
                    self.get_logger().info(
                        f'第 {channel} 路持续 {duration:.3f} 秒后已自动断开'
                    )
                else:
                    self.get_logger().error(
                        f'第 {channel} 路到期自动断开失败'
                    )
            except Exception as exc:
                self.get_logger().error(
                    f'第 {channel} 路到期自动断开异常：{exc}'
                )

        self._off_timers[channel] = self.create_timer(
            duration,
            auto_off_callback,
        )

    def _set_relay_callback(
        self,
        request: SetRelay.Request,
        response: SetRelay.Response,
    ) -> SetRelay.Response:
        """处理 /relay/set 服务请求。"""
        # uint8 允许数值 0，但继电器通道从 1 开始，因此需要额外校验。
        if request.channel < 1:
            response.success = False
            response.message = '通道号必须从 1 开始'
            self.get_logger().error(response.message)
            return response

        if not math.isfinite(request.duration) or request.duration < 0.0:
            response.success = False
            response.message = '持续时间必须是大于等于 0 的有限数值'
            self.get_logger().error(response.message)
            return response

        try:
            success = self._write_relay(request.channel, request.enabled)
        except Exception as exc:
            response.success = False
            response.message = f'继电器控制异常：{exc}'
            self.get_logger().error(response.message)
            return response

        if not success:
            response.success = False
            response.message = f'第 {request.channel} 路继电器控制失败'
            self.get_logger().error(response.message)
            return response

        # 当前通道收到新请求后，旧的自动断开任务不应继续生效。
        self._cancel_off_timer(request.channel)
        if request.enabled and request.duration > 0.0:
            self._schedule_auto_off(request.channel, request.duration)

        state = '吸合' if request.enabled else '断开'
        response.success = True
        if request.enabled and request.duration > 0.0:
            response.message = (
                f'第 {request.channel} 路继电器已吸合，'
                f'{request.duration:.3f} 秒后自动断开'
            )
        else:
            response.message = f'第 {request.channel} 路继电器已{state}'
        self.get_logger().info(response.message)
        return response

    def destroy_node(self):
        # 先停止全部定时器，防止退出过程中再次触发串口操作。
        for channel in tuple(self._off_timers):
            self._cancel_off_timer(channel)

        # 节点正常退出时断开本节点曾经吸合的全部通道。
        for channel in tuple(self._active_channels):
            try:
                self._write_relay(channel, False)
            except Exception as exc:
                self.get_logger().error(
                    f'退出时断开第 {channel} 路继电器失败：{exc}'
                )
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RelayController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        # 初始化失败（例如配置错误或串口依赖缺失）时给出明确日志。
        rclpy.logging.get_logger('relay_controller').fatal(
            f'继电器节点启动失败：{exc}'
        )
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
