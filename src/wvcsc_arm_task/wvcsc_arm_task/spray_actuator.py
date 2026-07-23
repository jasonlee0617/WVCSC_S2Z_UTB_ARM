# spray_actuator.py
"""
喷洒执行器 Action 服务器 (Spray Action Server, 仿真/实机共用)。

职责：
1. 实现 `/spray/execute` Action 服务，控制喷洒泵/阀。
2. 仿真时通过定时器模拟；实机时通过 GPIO/串口控制（替换 execute 循环中的 sleep）。
3. 使用 `SprayInterlock` 实现线程安全的喷洒并发控制与机械臂停止联锁。
4. 在喷洒期间通过 Feedback 实时反馈进度。
5. 支持 Action 取消请求和 `/motion_control/locked` 立即关喷洒。
"""

import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from wvcsc_interfaces.action import Spray
from wvcsc_interfaces.srv import SetRelay

from .core import SprayInterlock


class SprayActuator(Node):
    def __init__(self, **kwargs):
        super().__init__('wvcsc_spray_actuator', **kwargs)
        
        # 1. 声明所有 ROS2 参数，为后期替换真机驱动提供 YAML 配置接口
        self.declare_parameter('action_name', '/spray/execute')
        self.declare_parameter('active_topic', '/spray/simulated_active')
        self.declare_parameter('motion_locked_topic', '/motion_control/locked')
        self.declare_parameter('min_duration', 0.2)
        self.declare_parameter('max_duration', 10.0)
        self.declare_parameter('spray_mode', 'timer')
        self.declare_parameter('relay_service_name', '/relay/set')
        self.declare_parameter('relay_channel', 2)
        self.declare_parameter('relay_service_timeout_sec', 2.0)
        
        # 2. 实例化喷洒核心互锁模块 (纯逻辑安全闸门)
        self._interlock = SprayInterlock(
            self.get_parameter('min_duration').value,
            self.get_parameter('max_duration').value,
        )
        self._spray_mode = str(self.get_parameter('spray_mode').value)
        self._relay_service_name = str(self.get_parameter('relay_service_name').value)
        self._relay_channel = int(self.get_parameter('relay_channel').value)
        self._relay_service_timeout = float(
            self.get_parameter('relay_service_timeout_sec').value)
        self._relay_client = None
        if self._spray_mode == 'service':
            if self._relay_channel < 1:
                raise ValueError('relay_channel must be at least one')
            if self._relay_service_timeout <= 0.0:
                raise ValueError('relay_service_timeout_sec must be positive')
            self._relay_client = self.create_client(
                SetRelay, self._relay_service_name)
            self.get_logger().info(
                f'[SPRAY] service mode, relay service={self._relay_service_name} '
                f'channel={self._relay_channel}')

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
        request_started = time.monotonic()
        started = None
        duration = float(goal_handle.request.duration)
        
        pump_started = False
        try:
            started_ok, message = self._pump_on(duration)
            if not started_ok:
                result.error_code = Spray.Result.RELAY_FAILED
                result.message = message
                goal_handle.abort()
            else:
                pump_started = True
                started = time.monotonic()
                self._set_active(True)
                self.get_logger().info(
                    f'[SPRAY] mode={self._spray_mode} duration={duration:.1f}s')
                while True:
                    elapsed = time.monotonic() - started

                    if self._interlock.motion_locked:
                        result.error_code = Spray.Result.EMERGENCY_STOPPED
                        result.message = 'spray stopped because motion is locked'
                        goal_handle.abort()
                        break

                    if goal_handle.is_cancel_requested:
                        result.error_code = Spray.Result.CANCELED
                        result.message = 'spray canceled'
                        goal_handle.canceled()
                        break

                    if elapsed >= duration:
                        result.success = True
                        result.error_code = Spray.Result.OK
                        result.message = 'spray completed'
                        goal_handle.succeed()
                        self.get_logger().info('[SPRAY] 喷洒动作成功执行')
                        break

                    feedback = Spray.Feedback()
                    feedback.phase = Spray.Feedback.ACTIVE
                    feedback.elapsed = elapsed
                    feedback.progress = min(1.0, elapsed / duration)
                    goal_handle.publish_feedback(feedback)
                    time.sleep(min(0.02, duration - elapsed))
        finally:
            if pump_started:
                self._set_active(False)
            self._pump_off()
            self._interlock.release()
            
        result.actual_duration = time.monotonic() - (
            started if started is not None else request_started)
        return result

    def _on_motion_locked(self, message):
        """响应机械臂 stop/reset 锁，立即停止当前喷洒。"""
        self._interlock.set_motion_locked(message.data)
        if message.data:
            self._set_active(False)
            self._pump_off()

    def _set_active(self, active):
        """发布喷洒活跃状态，供任务管理器或 Web 界面监控。"""
        message = Bool()
        message.data = bool(active)
        self._active_pub.publish(message)

    def _relay_request(self, enabled, duration, wait):
        if self._spray_mode != 'service' or self._relay_client is None:
            return True, ''
        request = SetRelay.Request()
        request.channel = self._relay_channel
        request.enabled = bool(enabled)
        request.duration = float(duration)
        if not wait:
            self._relay_client.call_async(request)
            return True, ''

        deadline = time.monotonic() + self._relay_service_timeout
        while not self._relay_client.service_is_ready():
            if time.monotonic() >= deadline:
                return False, 'relay service is unavailable'
            time.sleep(0.02)
        future = self._relay_client.call_async(request)
        while not future.done():
            if time.monotonic() >= deadline:
                return False, 'relay service timed out'
            time.sleep(0.02)
        try:
            response = future.result()
        except Exception as error:
            return False, f'relay service failed: {error}'
        if response is None or not response.success:
            detail = '' if response is None else response.message
            return False, f'relay service rejected request: {detail}'
        return True, ''

    def _pump_on(self, duration):
        """Start the pump only after the real relay service confirms it."""
        return self._relay_request(True, duration, wait=True)

    def _pump_off(self):
        """Request an immediate off; the duration timeout remains a backup."""
        self._relay_request(False, 0.0, wait=False)

    def force_off(self):
        """
        强制关闭喷洒互锁与发布器（用于节点销毁前的深度清理）。
        """
        self._interlock.set_motion_locked(True)
        if self.context.ok():
            self._set_active(False)
            self._pump_off()


def main():
    rclpy.init()
    node = SprayActuator()
    
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
