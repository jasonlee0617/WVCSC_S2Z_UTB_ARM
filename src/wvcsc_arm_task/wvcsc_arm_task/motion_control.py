# motion_control.py
"""
Alicia-M 机械臂运动控制的安全仲裁节点 (Motion Control Node)。

核心功能：
1. 接收并执行系统级的运动控制指令：`stop`(急停), `reset`(复位至HOME), `resume`(解除锁定)。
2. 将当前的运动锁定状态 (`locked`) 通过 QoS 设置为 `TRANSIENT_LOCAL` 实时发布，
   以通知下游节点（如 `spray_task`）阻止新的运动规划。
3. 扮演“看门狗”角色，确保任何异常发生后的运动恢复均经过人工确认。
"""

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
    """
    标准化并校验接收到的运动控制指令。

    避免大小写和前后空格导致的误触发。
    """
    command = str(value).strip().lower()
    return command if command in ('stop', 'reset', 'resume') else None


class MotionControlNode(Node):
    def __init__(self):
        super().__init__('wvcsc_motion_control')

        # 1. 初始化线程安全的共享状态锁（MotionControlState）
        # 所有机械臂长时任务（如 `spray_task`）都会检查这个状态。
        self.state = MotionControlState()

        # 2. 实例化 AliciaMoveIt 运动适配器。
        # 注意：这里将 self.state 传入，使底层的 alicia_moveit 也能感知状态锁。
        self.arm, self._callback_group = create_alicia_moveit(self, self.state)

        # 3. 发布锁状态话题（Latch 模式）
        # 使用 `TRANSIENT_LOCAL` 持久化策略：后订阅该话题的节点（如 `spray_task`）
        # 能够立刻收到最新的“锁定”状态，而不会错过在它启动前就已经锁定的瞬间。
        self._locked_pub = self.create_publisher(
            Bool,
            '/motion_control/locked',
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )

        # 4. 订阅运动控制命令话题
        # 接收来自 Web 界面、遥控器或系统急停模块的强制命令。
        self.command_sub = self.create_subscription(
            String,
            '/motion_control/command',
            self._on_command,
            10,
            callback_group=self._callback_group,
        )

        # 5. 初始化时立即发布一次当前的锁定状态（默认未锁定）。
        self._publish_locked()

    def _publish_locked(self):
        """将状态机的锁定状态同步发布给所有下游节点。"""
        message = Bool()
        message.data = self.state.locked
        self._locked_pub.publish(message)

    def _on_command(self, message):
        """
        运动控制命令的回调函数。

        处理三种状态指令：
        - `stop`：立即锁定并取消所有运动。
        - `reset`：锁定状态，并异步在后台执行 HOME 位姿复位。
        - `resume`：仅在复位完成后，手动解除锁定。
        """
        command = normalize_command(message.data)
        if command is None:
            self.get_logger().warn(f'Ignoring unknown motion command: {message.data!r}')
            return

        if command == 'stop':
            # --------- 紧急停止指令 ---------
            self.state.stop()          # 状态机标记为锁定
            self._publish_locked()     # 广播锁定信号，阻止其他任务继续下发 /cmd_vel 或 MoveIt 动作
            self.arm.cancel()          # 立刻取消当前底层已发送的 MoveIt 轨迹
            self.get_logger().warn('Motion stopped; new goals are locked')

        elif command == 'reset':
            # --------- 复位指令 ---------
            # `begin_reset` 执行三个动作：1.设置状态锁定；2.尝试取消当前运动并等待完全停止；
            # 3.如果在超时时间内没能停止，则返回 False。
            if not begin_reset(self.state, self.arm):
                self._publish_locked()
                self.get_logger().error(
                    'Reset could not start or motion did not stop; motion remains locked')
                return
            
            self._publish_locked()
            
            # 启动一个守护线程执行机械臂回 HOME 的物理动作。
            # 这样做的目的是：即使 MoveIt 规划的 `perform_reset` 耗时较长（如 10s+），
            # 也不会卡死 `MotionControlNode` 的 ROS2 主回调线程，其他节点依然可以正常通讯。
            threading.Thread(target=self._reset, daemon=True).start()

        elif self.state.resume():
            # --------- 恢复指令 ---------
            # `resume()` 内部检查：只有 `_reset_in_progress` 为 False（即复位线程已完成）时，
            # 才会解除 `_locked` 锁定。
            self._publish_locked()
            self.get_logger().info('Motion lock released')
        else:
            self.get_logger().warn('Cannot resume while reset is in progress')

    def _reset(self):
        """
        在后台守护线程中执行的机械臂实际复位动作。
        
        它将调用 `alicia_moveit` 控制机械臂回到 `AliciaMoveIt.HOME` 位置。
        """
        # `perform_reset` 内部会自动完成：打开夹爪 -> 规划关节空间路径 -> 执行至 HOME。
        # 成功后状态设置为完成（finish_reset），但**依然处于锁定状态**，直到收到 `resume`。
        success = perform_reset(self.state, self.arm, AliciaMoveIt.HOME)
        
        if success:
            self.get_logger().info('Reset reached HOME; send resume to unlock motion')
        else:
            self.get_logger().error('Reset failed; motion remains locked')
        
        # 通知订阅者当前的锁定状态（通常仍在锁定状态）
        self._publish_locked()


def main():
    rclpy.init()
    node = MotionControlNode()
    
    # 使用多线程执行器（至少 2 个线程）。
    # 这能确保在 `_reset` 计算轨迹或执行时，`command_sub` 的回调依然能正常响应新的指令。
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