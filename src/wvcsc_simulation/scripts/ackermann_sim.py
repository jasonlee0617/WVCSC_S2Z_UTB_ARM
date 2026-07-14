#!/usr/bin/env python3
import math

import rclpy
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class AckermannSim(Node):
    UPDATE_PERIOD = 0.05
    COMMAND_TIMEOUT = 0.5

    def __init__(self):
        super().__init__('ackermann_sim')
        self.x = self.y = self.yaw = 0.0
        self.speed = self.yaw_rate = 0.0
        self.last_time = self.get_clock().now()
        self.last_command_time = None
        self.pending = False
        self.odom = self.create_publisher(Odometry, '/odom', 10)
        self.ekf_odom = self.create_publisher(Odometry, '/ekf_odom', 10)
        self.tf = TransformBroadcaster(self)
        self.client = self.create_client(SetEntityState, '/set_entity_state')
        self.create_subscription(Twist, '/cmd_vel', self.command, 10)
        self.create_timer(self.UPDATE_PERIOD, self.update)

    def command(self, message):
        self.speed = max(-0.8, min(0.8, message.linear.x))
        self.yaw_rate = max(-0.8, min(0.8, message.angular.z))
        self.last_command_time = self.get_clock().now()

    def update(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        if dt <= 0.0 or dt > 0.5:
            return

        if (self.last_command_time is not None and
                (now - self.last_command_time).nanoseconds * 1e-9 > self.COMMAND_TIMEOUT):
            self.speed = self.yaw_rate = 0.0

        self.yaw += self.yaw_rate * dt
        self.x += self.speed * math.cos(self.yaw) * dt
        self.y += self.speed * math.sin(self.yaw) * dt
        qz, qw = math.sin(self.yaw / 2.0), math.cos(self.yaw / 2.0)
        self.publish_state(now, qz, qw)
        self.set_gazebo_state(qz, qw)

    def publish_state(self, now, qz, qw):
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
        self.pending = False
        try:
            response = future.result()
            if not response.success:
                self.get_logger().error(response.status_message)
        except Exception as error:  # service failures must not stop odometry
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
