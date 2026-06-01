#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import yaml
from rclpy.qos import QoSProfile
from rclpy.duration import Duration
import math

class TraffiLightNav2(Node):
    '''
     * Author:zcj
     * Alter: zcj
     * Function：   __init__
     * Description：初始化函数
     * Input：无
     * Output：无
     * Return：无
    '''
    def __init__(self):
        super().__init__('path_follower')
        # 初始化Nav2导航器，用于路径规划和执行
        self.navigator = BasicNavigator()
        # 等待Nav2系统完全初始化（包括定位和规划器）
        self.get_logger().info('Waiting for Nav2 to initialize...')
        # self.navigator.waitUntilNav2Active()
        self.get_logger().info('Nav2 is ready!')

    
    '''
     * Author:zcj
     * Alter: zcj
     * Function：   split_path
     * Description：开始沿路径运动,使用navigator.followPath
     * Input：
     *   path: nav_msgs/msg/Path 路径消息
     * Output：无
     * Return：无
    '''
    def start_follow_path(self, path):
        self.get_logger().info(f'\nFollowing path...,node len: {len(path.poses)}')
        # 向Nav2发送路径跟随请求
        self.navigator.followPath(path)
        i = 0
        # 监控导航状态
        while not self.navigator.isTaskComplete():
            
            i += 1
            feedback = self.navigator.getFeedback()
            if feedback and i % 5 == 0:
                
                print('Estimated distance remaining to goal position: ' +
                      '{0:.3f}'.format(feedback.distance_to_goal) +
                      '\nCurrent speed of the robot: ' +
                      '{0:.3f}'.format(feedback.speed))
        # 获取并处理导航结果
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('路径跟随：Path following succeeded!')
        elif result == TaskResult.CANCELED:
            self.get_logger().info('路径跟随：Path following was canceled!')
        else:
            self.get_logger().info('路径跟随：Path following failed!')

    '''
     * Author:zcj
     * Alter: zcj
     * Function：   split_path2
     * Description：开始沿路径运动,使用navigator.followWaypoints
     * Input：
     *   path: nav_msgs/msg/Path 路径消息
     * Output：无
     * Return：无
    '''
    def start_follow_path2(self, path):
         # self.get_logger().info('\nFollowing path...,node len:',len(path.poses))
        self.get_logger().info(f'\nFollowing path...,node len: {len(path.poses)}')
        # 向Nav2发送路径跟随请求
        # self.navigator.followWaypoints(path.poses)
        poses = path.poses[::2]
        self.navigator.followWaypoints(poses)
        i = 0
        # 监控导航状态
        while not self.navigator.isTaskComplete():
            i += 1
            feedback = self.navigator.getFeedback()
            if feedback and feedback.current_waypoint > 0:
                self.get_logger().info(f'Following path: {feedback.current_waypoint}/{len(poses)} waypoints completed')
            if i%5==0:
                self.record_path_pub_.publish(path)
           
        # 获取并处理导航结果
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('Path following succeeded!')
        elif result == TaskResult.CANCELED:
            self.get_logger().info('Path following was canceled!')
        else:
            self.get_logger().info('Path following failed!')

    '''
     * Author:zcj
     * Alter: zcj
     * Function：   follow_path
     * Description：沿路径运动，先运动到起点
     * Input：
     *   path: nav_msgs/msg/Path 路径消息
     * Output：无
     * Return：无
    '''
    def follow_path(self, path):
        """执行路径跟随任务并监控状态"""
        if path is None:
            self.get_logger().error('No path to follow!')
            return
        
        self.record_path_pub_.publish(path)

        # Go to our demos first goal pose
        goal_pose = PoseStamped()
        # 获取路径的第一个点作为目标位置
        goal_pose = path.poses[0]
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        # 先导航到路径的第一个点（起点）
        # 让机器人导航到目标位置
        self.navigator.goToPose(goal_pose)

        i = 0
        # 检查导航任务是否完成
        while not self.navigator.isTaskComplete():
            i = i + 1

            feedback = self.navigator.getFeedback()
            if feedback and i % 5 == 0:

                print('Estimated time of arrival: ' + '{0:.0f}'.format(
                      Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                      + ' seconds.')
                self.record_path_pub_.publish(path)
                # Some navigation timeout to demo cancellation
                # 如果导航时间超过600秒，取消任务
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                    print('Navigation timed out, canceling task.')
                    navigator.cancelTask()
        # Do something depending on the return code
        # 根据导航结果打印信息
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('是否到达起点：Goal succeeded! ')
        elif result == TaskResult.CANCELED:
            print('是否到达起点：Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('是否到达起点：Goal failed!')
        else:
            print('是否到达起点：Goal has an invalid return status!')

        self.get_logger().info(f'\nFollowing path...,node len: {len(path.poses)}')

        print("开始沿路径行走")
        if(self.is_start_and_goal_same(path)):
            print("分两次行走")
            path1_msg,path2_msg = self.split_path(path)

            self.start_follow_path(path1_msg)
            self.start_follow_path(path2_msg)
        else:
            self.start_follow_path(path)

    
    def follow_path2(self, path):
        """执行路径跟随任务并监控状态"""
        if path is None:
            self.get_logger().error('No path to follow!')
            return
        
        self.record_path_pub_.publish(path)
        print("开始沿路径行走")
        self.start_follow_path2(path)

    def cancel_navigation(self):
        """取消当前导航任务"""

        # path = self.navigator.getPathThroughPoses()
        # self.get_logger().info('path :',path )

        self.navigator.cancelTask()
        self.get_logger().info('Navigation task canceled!')
    
    def _send_zero_velocity(self, duration=2.0):
        """发送零速度命令，持续指定时间（秒）"""
        twist = Twist()
        # 发送多次以确保机器人停止
        rate = self.create_rate(10)  # 10Hz
        for _ in range(int(duration * 10)):
            self.velocity_pub.publish(twist)
            rate.sleep()
'''
* Author:zcj
* Alter: zcj
* Function：   main
* Description：主函数
* Input：
*   args：运行传入参数
* Output：无
* Return：无
'''
def main(args=None):
    """主函数：初始化节点，加载路径并执行导航"""
    rclpy.init(args=args)
    follower = TraffiLightNav2()
    
    # # 加载预定义路径
    # path = follower.load_path()
    # # 执行路径跟随
    # follower.follow_path(path)
    # follower.follow_path2(path)

    follower.cancel_navigation()
    # 清理资源
    follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()