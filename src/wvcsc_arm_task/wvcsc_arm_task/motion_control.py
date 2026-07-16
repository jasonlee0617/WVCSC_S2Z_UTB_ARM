"""Stop/reset/resume controller for Alicia-M MoveIt motion."""

import threading

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from .alicia_moveit import AliciaMoveIt
from .motion_state import (
    MotionControlState,
    begin_reset,
    perform_reset,
)
from .node_parameters import create_alicia_moveit


def normalize_command(value):
    command = str(value).strip().lower()
    return command if command in ('stop', 'reset', 'resume') else None


class MotionControlNode(Node):
    def __init__(self):
        super().__init__('wvcsc_motion_control')
        self.state = MotionControlState()
        self.arm, self._callback_group = create_alicia_moveit(self, self.state)
        self._locked_pub = self.create_publisher(
            Bool,
            '/motion_control/locked',
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self.command_sub = self.create_subscription(
            String,
            '/motion_control/command',
            self._on_command,
            10,
            callback_group=self._callback_group,
        )
        self._publish_locked()

    def _publish_locked(self):
        message = Bool()
        message.data = self.state.locked
        self._locked_pub.publish(message)

    def _on_command(self, message):
        command = normalize_command(message.data)
        if command is None:
            self.get_logger().warn(f'Ignoring unknown motion command: {message.data!r}')
            return
        if command == 'stop':
            self.state.stop()
            self._publish_locked()
            self.arm.cancel()
            self.get_logger().warn('Motion stopped; new goals are locked')
        elif command == 'reset':
            if not begin_reset(self.state, self.arm):
                self._publish_locked()
                self.get_logger().error(
                    'Reset could not start or motion did not stop; motion remains locked')
                return
            self._publish_locked()
            threading.Thread(target=self._reset, daemon=True).start()
        elif self.state.resume():
            self._publish_locked()
            self.get_logger().info('Motion lock released')
        else:
            self.get_logger().warn('Cannot resume while reset is in progress')

    def _reset(self):
        success = perform_reset(self.state, self.arm, AliciaMoveIt.HOME)
        if success:
            self.get_logger().info('Reset reached HOME; send resume to unlock motion')
        else:
            self.get_logger().error('Reset failed; motion remains locked')
        self._publish_locked()


def main():
    rclpy.init()
    node = MotionControlNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
