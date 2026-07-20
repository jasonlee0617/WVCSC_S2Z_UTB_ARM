#!/usr/bin/env python3
# ackermann_sim.py
# ============================================================================
# 阿克曼运动学仿真里程计节点 (Ackermann Kinematic Simulator)
# ============================================================================
#
# 作用：
# 1. 接收 `/cmd_vel` (线速度/角速度) 指令。
# 2. 基于阿克曼运动学模型进行积分，计算当前车辆在 `odom` 坐标系下的 (x, y, yaw)。
# 3. 发布 `/odom` 和 `/ekf_odom` 里程计消息，以及 `odom -> base_footprint` TF 变换。
# 4. 通过调用 Gazebo 的 `/set_entity_state` 服务，强制使仿真模型与内部积分同步移动。
#
# 设计特点：
# - 使用“软实时 + 异步服务调用”策略，即使 `/set_entity_state` 调用略有延迟，
#   也不会阻塞里程计更新的 20 Hz 循环。
# - 使用独立的 `command_timeout` 确保当 `/cmd_vel` 停止发送时，车辆能尽快刹车。
#

import math

import rclpy
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class AckermannSim(Node):
    # 更新周期 50ms (20Hz)
    UPDATE_PERIOD = 0.05

    def __init__(self):
        super().__init__('ackermann_sim')
        self.declare_parameter('wheel_base', 0.67)
        self.declare_parameter('max_steering_angle', 0.48)
        self.declare_parameter('max_linear_speed', 0.8)
        self.declare_parameter('command_timeout', 0.5)
        self.wheel_base = float(self.get_parameter('wheel_base').value)
        self.max_steering_angle = float(
            self.get_parameter('max_steering_angle').value)
        self.max_linear_speed = float(
            self.get_parameter('max_linear_speed').value)
        self.command_timeout = float(
            self.get_parameter('command_timeout').value)
        
        if (self.wheel_base <= 0.0 or
                not 0.0 < self.max_steering_angle < math.pi / 2.0):
            raise ValueError('invalid Ackermann geometry parameters')

        # 初始状态
        self.x = self.y = self.yaw = 0.0
        self.speed = self.yaw_rate = 0.0
        self.last_time = self.get_clock().now()
        self.last_command_time = None
        self.pending = False  # 标记是否有正在等待回复的 /set_entity_state 服务调用

        # 发布器与订阅器
        self.odom = self.create_publisher(Odometry, '/odom', 10)
        self.ekf_odom = self.create_publisher(Odometry, '/ekf_odom', 10)
        self.tf = TransformBroadcaster(self)
        self.client = self.create_client(SetEntityState, '/set_entity_state')
        self.create_subscription(Twist, '/cmd_vel', self.command, 10)
        self.create_timer(self.UPDATE_PERIOD, self.update)
        self.publish_state(self.last_time, 0.0, 1.0)

    def command(self, message):
        """接收 `/cmd_vel` 速度指令。"""
        self.speed = max(
            -self.max_linear_speed,
            min(self.max_linear_speed, message.linear.x),
        )
        # 当线速度接近0时，直接清零角速度以避免原地微抖
        if abs(self.speed) < 1e-4:
            self.yaw_rate = 0.0
        else:
            # 阿克曼转向角与角速度的关系：yaw_rate = (v * tan(steering_angle)) / wheel_base
            max_yaw_rate = (
                abs(self.speed) * math.tan(self.max_steering_angle)
                / self.wheel_base
            )
            self.yaw_rate = max(
                -max_yaw_rate, min(max_yaw_rate, message.angular.z))
        self.last_command_time = self.get_clock().now()

    def update(self):
        """定时更新车辆状态、发布里程计、同步 Gazebo 模型。"""
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        # 防止 Gazebo 卡顿时 dt 过大导致状态爆炸，拒绝大于 0.5s 的步长
        if dt <= 0.0 or dt > 0.5:
            return

        # 指令超时检查：如果超过 `command_timeout` 没收到新的 cmd_vel，强制刹车
        if (self.last_command_time is not None and
                (now - self.last_command_time).nanoseconds * 1e-9
                > self.command_timeout):
            self.speed = self.yaw_rate = 0.0

        # 积分更新位置
        self.yaw += self.yaw_rate * dt
        self.x += self.speed * math.cos(self.yaw) * dt
        self.y += self.speed * math.sin(self.yaw) * dt
        qz, qw = math.sin(self.yaw / 2.0), math.cos(self.yaw / 2.0)
        self.publish_state(now, qz, qw)
        self.set_gazebo_state(qz, qw)

    def publish_state(self, now, qz, qw):
        """发布 ROS 标准 `Odometry` 和 TF。"""
        message = Odometry()
        message.header.stamp = now.to_msg()
        message.header.frame_id = 'odom'
        message.child_frame_id = 'base_footprint'
        message.pose.pose.position.x = self.x
        message.pose.pose.position.y = self.y
        message.pose.pose.orientation.z = qz
        message.pose.pose.orientation.w = qw
        message.twist.twist.linear.x = self.speed
        message.twist.twist.angular.z = self.yaw_rate
        self.odom.publish(message)
        self.ekf_odom.publish(message)

        transform = TransformStamped()
        transform.header = message.header
        transform.child_frame_id = message.child_frame_id
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.rotation = message.pose.pose.orientation
        self.tf.sendTransform(transform)

    def set_gazebo_state(self, qz, qw):
        """
        异步设置 Gazebo 实体状态。
        使用异步调用，即便 Gazebo 物理引擎响应慢，也不会阻塞里程计的高频发布。
        """
        if self.pending or not self.client.service_is_ready():
            return
        state = EntityState()
        state.name = 'wvcsc_utb_alicia'
        state.pose.position.x = self.x
        state.pose.position.y = self.y
        state.pose.orientation.z = qz
        state.pose.orientation.w = qw
        state.twist.linear.x = self.speed * math.cos(self.yaw)
        state.twist.linear.y = self.speed * math.sin(self.yaw)
        state.twist.angular.z = self.yaw_rate
        state.reference_frame = 'world'
        request = SetEntityState.Request(state=state)
        self.pending = True
        self.client.call_async(request).add_done_callback(self.response_received)

    def response_received(self, future):
        """异步服务调用的回调。"""
        self.pending = False
        try:
            response = future.result()
            if not response.success:
                self.get_logger().error(response.status_message)
        except Exception as error:  # 即使服务失败，也绝不能阻挡高优先级的里程计更新
            self.get_logger().error(f'Failed to set Gazebo vehicle state: {error}')


def main():
    rclpy.init()
    node = AckermannSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
            rclpy.try_shutdown()
        except KeyboardInterrupt:
            pass


if __name__ == '__main__':
    main()