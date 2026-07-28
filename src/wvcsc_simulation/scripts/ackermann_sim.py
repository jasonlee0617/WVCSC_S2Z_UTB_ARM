#!/usr/bin/env python3
# ackermann_sim.py
# ============================================================================
# 阿克曼运动学仿真里程计节点 (Ackermann Kinematic Simulator)
# ============================================================================
#
# 作用：
# 1. 默认接收标准 ROS `/cmd_vel`（线速度/偏航角速度）指令，并按车辆的
#    最大前轮转角限制可实现曲率。
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
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64
from tf2_ros import TransformBroadcaster

from wvcsc_interfaces.msg import MissionPlan, MissionStatus
from wvcsc_simulation.ackermann_math import (
    point_to_polyline_distance,
    point_to_segment_distance,
    yaw_rate_from_steering,
    yaw_rate_from_twist,
)


class AckermannSim(Node):
    # 更新周期 50ms (20Hz)
    UPDATE_PERIOD = 0.05

    def __init__(self):
        super().__init__('ackermann_sim')
        self.declare_parameter('wheel_base', 0.82)
        self.declare_parameter('max_steering_angle', 0.48)
        self.declare_parameter('max_linear_speed', 0.8)
        self.declare_parameter('command_timeout', 0.5)
        # Nav2 使用标准 Twist：angular.z 为偏航角速度。保留 steering_angle
        # 仅供需要复现实车非标准接口的显式兼容测试。
        self.declare_parameter('cmd_angular_mode', 'yaw_rate')
        self.declare_parameter('executed_path_topic', '/vehicle/executed_path')
        self.declare_parameter('executed_path_min_distance_m', 0.02)
        self.declare_parameter('executed_path_max_poses', 5000)
        self.wheel_base = float(self.get_parameter('wheel_base').value)
        self.max_steering_angle = float(
            self.get_parameter('max_steering_angle').value)
        self.max_linear_speed = float(
            self.get_parameter('max_linear_speed').value)
        self.command_timeout = float(
            self.get_parameter('command_timeout').value)
        self.cmd_angular_mode = str(
            self.get_parameter('cmd_angular_mode').value).strip().lower()
        self.executed_path_min_distance = float(
            self.get_parameter('executed_path_min_distance_m').value)
        self.executed_path_max_poses = int(
            self.get_parameter('executed_path_max_poses').value)
        
        if (self.wheel_base <= 0.0 or
                not 0.0 < self.max_steering_angle < math.pi / 2.0):
            raise ValueError('invalid Ackermann geometry parameters')
        if self.command_timeout <= self.UPDATE_PERIOD:
            raise ValueError(
                'command_timeout must exceed the Ackermann update period '
                f'({self.UPDATE_PERIOD:.3f}s)')
        if self.cmd_angular_mode not in {'steering_angle', 'yaw_rate'}:
            raise ValueError(
                'cmd_angular_mode must be steering_angle or yaw_rate')
        if (self.executed_path_min_distance <= 0.0 or
                self.executed_path_max_poses <= 0):
            raise ValueError('invalid executed-path parameters')

        # 初始状态
        self.x = self.y = self.yaw = 0.0
        self.speed = self.yaw_rate = 0.0
        self.last_time = self.get_clock().now()
        self.last_command_time = None
        self.pending = False  # 标记是否有正在等待回复的 /set_entity_state 服务调用
        self._mission_id = ''
        self._mission_index = None
        self._route_home = None
        self._route_goals = ()
        self._controller_path = ()
        self._last_path_position = None
        self.executed_path = Path()
        self.executed_path.header.frame_id = 'odom'

        # 发布器与订阅器
        self.odom = self.create_publisher(Odometry, '/odom', 10)
        self.ekf_odom = self.create_publisher(Odometry, '/ekf_odom', 10)
        path_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.executed_path_publisher = self.create_publisher(
            Path, str(self.get_parameter('executed_path_topic').value), path_qos)
        self.route_cross_track_publisher = self.create_publisher(
            Float64, '/vehicle/route_cross_track_error', 10)
        self.controller_path_error_publisher = self.create_publisher(
            Float64, '/vehicle/controller_path_error', 10)
        self.tf = TransformBroadcaster(self)
        self.client = self.create_client(SetEntityState, '/set_entity_state')
        self.create_subscription(Twist, '/cmd_vel', self.command, 10)
        self.create_subscription(
            MissionStatus, '/mission/status', self._on_mission_status, path_qos)
        self.create_subscription(
            MissionPlan, '/mission/plan', self._on_mission_plan, path_qos)
        self.create_subscription(Path, '/plan', self._on_controller_path, 10)
        self.create_timer(self.UPDATE_PERIOD, self.update)
        self.publish_state(self.last_time, 0.0, 1.0)

    def command(self, message):
        """接收 Nav2 的速度指令。"""
        self.speed = max(
            -self.max_linear_speed,
            min(self.max_linear_speed, message.linear.x),
        )
        # 当线速度接近0时，直接清零角速度以避免原地微抖。
        if abs(self.speed) < 1e-4:
            self.yaw_rate = 0.0
        elif self.cmd_angular_mode == 'steering_angle':
            steering_angle = max(
                -self.max_steering_angle,
                min(self.max_steering_angle, message.angular.z),
            )
            self.yaw_rate = yaw_rate_from_steering(
                self.speed, steering_angle, self.wheel_base)
        else:
            # 标准 ROS Twist 兼容模式：angular.z 表示偏航角速度。
            self.yaw_rate = yaw_rate_from_twist(
                self.speed, message.angular.z, self.wheel_base,
                self.max_steering_angle)
        self.last_command_time = self.get_clock().now()

    def _on_mission_status(self, message):
        """新任务开始时清空轨迹，避免不同任务的路线混在同一条 Path 中。"""
        mission_id = str(message.mission_id).strip()
        if mission_id and mission_id != self._mission_id:
            self._mission_id = mission_id
            self._mission_index = None
            self._last_path_position = None
            self.executed_path = Path()
            self.executed_path.header.frame_id = 'odom'
            self.executed_path_publisher.publish(self.executed_path)
        if mission_id == self._mission_id:
            self._mission_index = int(message.current_index)

    @staticmethod
    def _xy_from_pose(pose):
        return float(pose.position.x), float(pose.position.y)

    def _on_mission_plan(self, message):
        """Cache the vehicle-base route that Qt submitted to MissionManager."""
        mission_id = str(message.mission_id).strip()
        if not mission_id:
            return
        self._mission_id = mission_id
        self._route_home = self._xy_from_pose(message.home_pose)
        self._route_goals = tuple(
            self._xy_from_pose(point.docking_pose) for point in message.points)

    def _on_controller_path(self, message):
        """Cache Nav2's active detour path for comparison with the cyan route."""
        self._controller_path = tuple(
            (float(pose.pose.position.x), float(pose.pose.position.y))
            for pose in message.poses)

    def _route_cross_track_error(self):
        index = self._mission_index
        if (index is None or self._route_home is None or
                index < 0 or index >= len(self._route_goals)):
            return math.nan
        start = self._route_home if index == 0 else self._route_goals[index - 1]
        return point_to_segment_distance(
            self.x, self.y, start, self._route_goals[index])

    def _publish_route_tracking_errors(self):
        """Publish route and active-plan errors without changing Nav2 control."""
        self.route_cross_track_publisher.publish(Float64(
            data=self._route_cross_track_error()))
        self.controller_path_error_publisher.publish(Float64(
            data=point_to_polyline_distance(
                self.x, self.y, self._controller_path)))

    def _append_executed_path(self, message):
        """只在任务执行期间记录实际里程计路径，并按距离进行抽样。"""
        if not self._mission_id:
            return
        position = message.pose.pose.position
        previous = self._last_path_position
        if previous is not None and math.hypot(
                position.x - previous[0], position.y - previous[1]
        ) < self.executed_path_min_distance:
            return
        pose = PoseStamped()
        pose.header = message.header
        pose.pose = message.pose.pose
        self.executed_path.header = message.header
        self.executed_path.poses.append(pose)
        if len(self.executed_path.poses) > self.executed_path_max_poses:
            del self.executed_path.poses[:-self.executed_path_max_poses]
        self._last_path_position = (position.x, position.y)
        self.executed_path_publisher.publish(self.executed_path)

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
        self._append_executed_path(message)
        self._publish_route_tracking_errors()

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
