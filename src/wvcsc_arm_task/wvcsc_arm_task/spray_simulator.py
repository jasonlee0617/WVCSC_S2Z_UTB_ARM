# spray_simulator.py
"""
仿真环境下的喷洒执行器 Action 服务器 (Simulation-only Spray Action Server)。

职责：
1. 实现 `/spray/execute` Action 服务，模拟喷洒泵/阀的行为。
2. 使用 `SprayInterlock` 实现线程安全的喷洒并发控制与机械臂停止联锁。
3. 在喷洒期间通过 Feedback 实时反馈进度。
4. 支持 Action 取消请求和 `/motion_control/locked` 立即关喷洒。
"""

import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from wvcsc_interfaces.action import Spray

from .core import SprayInterlock


class SpraySimulator(Node):
    def __init__(self, **kwargs):
        super().__init__('wvcsc_spray_simulator', **kwargs)
        
        # 1. 声明所有 ROS2 参数，为后期替换真机驱动提供 YAML 配置接口
        self.declare_parameter('action_name', '/spray/execute')
        self.declare_parameter('active_topic', '/spray/simulated_active')
        self.declare_parameter('motion_locked_topic', '/motion_control/locked')
        self.declare_parameter('min_duration', 0.2)
        self.declare_parameter('max_duration', 10.0)
        
        # 2. 实例化喷洒核心互锁模块 (纯逻辑安全闸门)
        self._interlock = SprayInterlock(
            self.get_parameter('min_duration').value,
            self.get_parameter('max_duration').value,
        )

        # 3. 设置 Latch (持久化) QoS 配置。
        # 使用 `TRANSIENT_LOCAL` 确保下游节点能立刻得到当前活跃或锁定状态。
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        
        # 4. 发布当前喷洒激活状态（供任务管理器判断忙闲）
        self._active_pub = self.create_publisher(
            Bool, str(self.get_parameter('active_topic').value), latched)
            
        # 5. 订阅机械臂运动锁。stop/reset 时立即关闭喷洒。
        self.create_subscription(
            Bool,
            str(self.get_parameter('motion_locked_topic').value),
            self._on_motion_locked,
            latched,
        )
        
        # 6. 创建 Action Server
        self._server = ActionServer(
            self,
            Spray,
            str(self.get_parameter('action_name').value),
            execute_callback=self._execute,
            goal_callback=self._goal,
            cancel_callback=self._cancel,
        )
        self._set_active(False)

    def _goal(self, request):
        """
        Action Goal 回调：在开始执行前进行参数合法性校验，并尝试获取互锁控制权。
        """
        # 使用 core.py 中的 SprayInterlock 进行参数验证
        error = self._interlock.validate(
            request.mission_id,
            request.tree_id,
            float(request.duration),
            request.mode,
        )
        if error:
            self.get_logger().warn(f'[SPRAY] rejected goal: {error}')
            return GoalResponse.REJECT
            
        # 尝试锁住喷洒控制权；如果被其他目标占用或机械臂锁定，则拒绝请求。
        if not self._interlock.claim():
            self.get_logger().warn('[SPRAY] rejected goal: busy or motion locked')
            return GoalResponse.REJECT
            
        return GoalResponse.ACCEPT

    @staticmethod
    def _cancel(_goal_handle):
        """接收客户端的取消请求，允许任务取消。"""
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle):
        """
        Action 执行主循环。
        
        由于是仿真，这里仅进行睡眠模拟；未来替换为真机驱动时，
        只需将 `time.sleep` 替换为真实的 GPIO 控制代码即可。
        在真机实施时，需根据实际硬件的响应时间，改用 `wait_for_interrupt` 或轮询。
        """
        result = Spray.Result()
        started = time.monotonic()
        duration = float(goal_handle.request.duration)
        
        # 开泵标志与反馈
        self._set_active(True)
        self.get_logger().info(
            f'[SPRAY] 喷洒动作进行中......duration={duration:.1f}s')

        try:
            while True:
                elapsed = time.monotonic() - started
                
                # [1] 机械臂 stop/reset 时立即关闭喷洒。
                if self._interlock.motion_locked:
                    result.error_code = Spray.Result.EMERGENCY_STOPPED
                    result.message = 'spray stopped because motion is locked'
                    goal_handle.abort()
                    break
                
                # [2] 安全检查：检测 Action 是否被外部取消
                if goal_handle.is_cancel_requested:
                    result.error_code = Spray.Result.CANCELED
                    result.message = 'spray canceled'
                    goal_handle.canceled()
                    break
                
                # [3] 判定喷洒时长是否达到目标
                if elapsed >= duration:
                    result.success = True
                    result.error_code = Spray.Result.OK
                    result.message = 'simulated spray completed'
                    goal_handle.succeed()
                    self.get_logger().info('[SPRAY] 喷洒动作成功执行')
                    break
                
                # [4] 发布过程反馈 (Progress Feedback)
                feedback = Spray.Feedback()
                feedback.phase = Spray.Feedback.ACTIVE
                feedback.elapsed = elapsed
                feedback.progress = min(1.0, elapsed / duration)
                goal_handle.publish_feedback(feedback)
                
                # 仿真休眠：通过短时 sleep 模拟硬件响应延迟
                # 使用 min 防止负值，确保循环在达到目标时准确退出
                time.sleep(min(0.02, duration - elapsed))
                
        finally:
            # [5] 结束善后：无论成功/失败/取消/锁定，都必须关泵并释放互锁。
            self._set_active(False)
            self._interlock.release()
            
        result.actual_duration = time.monotonic() - started
        return result

    def _on_motion_locked(self, message):
        """响应机械臂 stop/reset 锁，立即停止当前喷洒。"""
        self._interlock.set_motion_locked(message.data)
        if message.data:
            self._set_active(False)

    def _set_active(self, active):
        """发布喷洒活跃状态，供任务管理器或 Web 界面监控。"""
        message = Bool()
        message.data = bool(active)
        self._active_pub.publish(message)

    def force_off(self):
        """
        强制关闭喷洒互锁与发布器（用于节点销毁前的深度清理）。
        """
        self._interlock.set_motion_locked(True)
        if self.context.ok():
            self._set_active(False)


def main():
    rclpy.init()
    node = SpraySimulator()
    
    # 使用多线程执行器，同时支持 Action 主循环的高延迟（睡眠模拟）
    # 和急停订阅回调的低延迟响应。
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # 优雅关闭：强制停泵，释放资源
        node.force_off()
        executor.shutdown(timeout_sec=2.0)
        node._server.destroy()
        node.destroy_node()
        rclpy.try_shutdown()
