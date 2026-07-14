"""MoveIt-based simulated spraying sequence for Alicia-M."""

import threading

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .motion_control import normalize_command
from .motion_state import MotionControlState
from .node_parameters import create_alicia_moveit


class SprayTask(Node):
    HOME = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    OBSERVE_LEFT = [0.65, -1.35, -1.05, 0.0, -0.75, 0.65]
    OBSERVE_RIGHT = [-0.65, -1.35, -1.05, 0.0, -0.75, -0.65]

    def __init__(self):
        super().__init__('wvcsc_spray_task')
        self.declare_parameter('spray_side', 'left')
        self.declare_parameter('spray_duration', 2.0)
        self.state = MotionControlState()
        self.arm, self._callback_group = create_alicia_moveit(self, self.state)
        self.service = self.create_service(
            Trigger,
            '/arm/execute_spray',
            self.execute,
            callback_group=self._callback_group,
        )
        self.command_sub = self.create_subscription(
            String,
            '/motion_control/command',
            self._on_motion_command,
            10,
            callback_group=self._callback_group,
        )
        self._abort = threading.Event()
        self._busy_mutex = threading.Lock()
        self._busy = False

    def _on_motion_command(self, message):
        command = normalize_command(message.data)
        if command in ('stop', 'reset'):
            self.state.stop()
            self._abort.set()
            self.arm.cancel()
        elif command == 'resume':
            self.state.resume()
            if not self._busy:
                self._abort.clear()

    def execute(self, _request, response):
        with self._busy_mutex:
            if self._busy:
                response.message = 'spray task is already running'
                return response
            if self.state.locked:
                response.message = 'motion is locked; send resume before starting'
                return response
            self._busy = True

        side = str(self.get_parameter('spray_side').value).lower()
        self._abort.clear()
        threading.Thread(
            target=self.run_sequence, args=(side,), daemon=True).start()
        response.success = True
        response.message = f'{side} spray sequence accepted'
        return response

    def _move(self, positions):
        if self._abort.is_set() or not self.arm.move_joints(positions):
            raise RuntimeError('arm motion was stopped or failed')

    def run_sequence(self, side):
        observe = self.OBSERVE_RIGHT if side == 'right' else self.OBSERVE_LEFT
        try:
            self.get_logger().info('Moving Alicia-M to observation pose')
            self._move(observe)
            duration = float(self.get_parameter('spray_duration').value)
            self.get_logger().info(f'Simulated spraying for {duration:.1f} s')
            if self._abort.wait(max(0.0, duration)):
                raise RuntimeError('spraying was stopped')
            self.get_logger().info('Returning Alicia-M to HOME')
            self._move(self.HOME)
        except RuntimeError as error:
            self.get_logger().error(str(error))
        finally:
            with self._busy_mutex:
                self._busy = False


def main():
    rclpy.init()
    node = SprayTask()
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

