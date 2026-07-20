"""Optional TTY front-end for the existing WVCSC motion-control interfaces."""

import select
import sys
import termios
import time
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


def command_for_key(key):
    """Map a single key to an arm command or controlled-abort request."""
    return {
        ' ': 'stop',
        'h': 'reset',
        'r': 'resume',
        'x': 'controlled_abort',
    }.get(key)


class MotionControlKeyboard(Node):
    """Publish stop/reset/resume commands without duplicating safety state logic."""

    def __init__(self):
        super().__init__('wvcsc_motion_control_keyboard')
        # ROS 2 publisher already uses reliable QoS.  Repeating ``reset`` can
        # race the single-flight HOME state machine, so safety commands are
        # emitted exactly once by default.  The parameter remains available
        # for a site-specific lossy transport adapter if one is added later.
        self.declare_parameter('command_burst_count', 1)
        self.declare_parameter('command_burst_period_sec', 0.01)
        self.declare_parameter('keyboard_poll_period_sec', 0.01)
        self._command_pub = self.create_publisher(
            String, '/motion_control/command', 10)
        self._abort_client = self.create_client(
            Trigger, '/safety/controlled_abort')
        self._stream = None
        self._fd = None
        self._old_settings = None
        self._opened_tty = None
        self._configure_tty()
        if self._stream is None:
            self.get_logger().warn('No TTY available; keyboard input is disabled')
            return
        period = max(
            0.005,
            float(self.get_parameter('keyboard_poll_period_sec').value))
        self.create_timer(period, self._poll)
        self.get_logger().info(
            'Keys: SPACE stop, h reset/HOME, r resume, x controlled abort')

    def _configure_tty(self):
        try:
            stream = sys.stdin if sys.stdin.isatty() else open('/dev/tty', 'r')
            opened = None if stream is sys.stdin else stream
            fd = stream.fileno()
            old = termios.tcgetattr(fd)
            tty.setcbreak(fd)
        except (OSError, termios.error):
            try:
                opened.close()
            except (AttributeError, OSError):
                pass
            return
        self._stream = stream
        self._fd = fd
        self._old_settings = old
        self._opened_tty = opened

    def _publish_command(self, command):
        count = max(1, int(self.get_parameter('command_burst_count').value))
        period = max(
            0.0, float(self.get_parameter('command_burst_period_sec').value))
        for index in range(count):
            self._command_pub.publish(String(data=command))
            if index + 1 < count and period > 0.0:
                time.sleep(period)
        self.get_logger().warn(f'motion command sent: {command}')

    def _poll(self):
        try:
            readable, _, _ = select.select([self._stream], [], [], 0.0)
        except (OSError, ValueError):
            return
        if not readable:
            return
        command = command_for_key(self._stream.read(1))
        if command is None:
            return
        if command == 'controlled_abort':
            # 无论安全协调服务是否已启动，先直接触发机械臂 stop。独立手眼
            # 标定不会启动底盘 safety 节点，因此 x 仍必须具有确定的停臂
            # 效果；完整实机系统中再由 controlled_abort 协调底盘与 HOME。
            self._publish_command('stop')
            if not self._abort_client.service_is_ready():
                self.get_logger().warn(
                    '/safety/controlled_abort is not ready; arm stop sent only')
                return
            self._abort_client.call_async(Trigger.Request())
            self.get_logger().warn('controlled abort requested')
            return
        self._publish_command(command)

    def destroy_node(self):
        if self._fd is not None and self._old_settings is not None:
            try:
                termios.tcsetattr(
                    self._fd, termios.TCSADRAIN, self._old_settings)
            except termios.error:
                pass
        if self._opened_tty is not None:
            try:
                self._opened_tty.close()
            except OSError:
                pass
        super().destroy_node()


def main():
    rclpy.init()
    node = MotionControlKeyboard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
