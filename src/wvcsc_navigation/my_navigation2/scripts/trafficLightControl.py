#!/usr/bin/python3

import rclpy
from nav2_msgs.action import FollowPath
from action_msgs.msg import GoalStatus
from action_msgs.msg import GoalStatusArray

from rclpy.action import ActionClient
from rclpy.action import ActionServer
from nav2_msgs.action import NavigateToPose, FollowPath
from geometry_msgs.msg import Twist
from rclpy.node import Node
import argparse
import getch
import threading
from std_msgs.msg import Int32  # 使用Int32消息类型


class TrafficLightNavigator(Node):
    def __init__(self):
        super().__init__('traffic_light_navigator')
        self.traffic_sub = self.create_subscription(
            Int32, '/light ', self.traffic_callback, 10)
        

        self.navigate_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.current_goal = None
        self.is_navigating = False
        
    def traffic_callback(self, ID):
        if msg.data == 0 and self.is_navigating:
            self.get_logger().info("红灯！暂停导航")
            self.cancel_navigation()
        elif msg.data == 1 and not self.is_navigating and self.current_goal:
            self.get_logger().info("绿灯！恢复导航")
            self.send_goal(self.current_goal)
    
    def send_goal(self, goal_msg):
        self.current_goal = goal_msg
        self.navigate_client.wait_for_server()
        self.navigate_client.send_goal_async(goal_msg)
        self.is_navigating = True
        
    def cancel_navigation(self):
        if self.is_navigating:
            # 取消当前导航
            self.navigate_client.cancel_all_goals()
            self.is_navigating = False

class NavigationController(Node):
    def __init__(self):
        super().__init__('navigation_controller')
        # 创建用于取消导航的action客户端
        self.navigate_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # 创建用于控制速度的发布者
        self.velocity_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self.goal_server = ActionServer(
            self,
            NavigateToPose,
            '/navigate_to_pose',
            # '/intercepted_navigate_to_pose',  # 修改服务器名称
            self.execute_callback)
        self.goal_handle = None  # 用于存储当前的goal句柄
        # 监听Nav2的状态
        # 订阅导航状态话题
        self.status_sub = self.create_subscription(
            GoalStatusArray,
            '/navigate_to_pose/_action/status',
            self.status_callback,
            10)
        self.gPose = None  # 用于存储目标位置

        threading.Thread(target=self.getKey,args=()).start()


    def execute_callback(self, goal_handle):
        self.goal_handle = goal_handle
        goal_pose = goal_handle.request.pose
        x = goal_pose.pose.position.x
        y = goal_pose.pose.position.y
        self.get_logger().info(f"拦截到导航目标: x={x:.2f}, y={y:.2f}")
        self.gPose = goal_pose
        
        return NavigateToPose.Result()
        
    def pause_navigation(self):
        """暂停导航并保持机器人静止"""
        try:
            self.get_logger().info('请求暂停导航...') 
            # 1. 尝试取消当前导航目标
            if self.navigate_client.wait_for_server(timeout_sec=2.0):
                # # 获取当前活动的goal句柄（实际应用中可能需要存储goal_id）
                # # 这里使用简化方法：发送空目标并立即取消
                # goal_msg = NavigateToPose.Goal()
                # target_pose = goal_msg.pose
                # self.get_logger().info(f'终点目标坐标: x={target_pose.pose.position.x}, y={target_pose.pose.position.y}, z={target_pose.pose.position.z}')

                # goal_future = self.navigate_client.send_goal_async(goal_msg)
                # # 等待目标发送完成
                # rclpy.spin_until_future_complete(self, goal_future)
                # goal_handle = goal_future.result()
                
                if self.goal_handle is not None:
                    # 取消目标
                    cancel_future = self.goal_handle.cancel_goal_async()
                    rclpy.spin_until_future_complete(self, cancel_future)
                    self.get_logger().info('导航目标已取消')
                else:
                    self.get_logger().warn('无法获取当前导航目标句柄')
            else:
                self.get_logger().warn('navigate_to_pose服务不可用，尝试直接控制速度')

            # 2. 发送零速度命令，确保机器人停止
            self._send_zero_velocity()
        except Exception as e:
            self.get_logger().error(f'暂停导航时出错: {str(e)}')
        
        

        
    def resume_navigation(self):
        """恢复导航到指定目标"""
        self.get_logger().info('请求恢复导航...')
        
        if self.navigate_client.wait_for_server(timeout_sec=2.0):
            if self.gPose is not None:
                goal_msg = NavigateToPose.Goal()
                goal_msg.pose = self.gPose
                
                goal_future = self.navigate_client.send_goal_async(goal_msg)
                rclpy.spin_until_future_complete(self, goal_future)
                
                if goal_future.result():
                    self.get_logger().info('导航已恢复')
                    return True
                else:
                    self.get_logger().error('恢复导航失败')
                    return False
        else:
            self.get_logger().error('navigate_to_pose服务不可用')
            return False
            
    def _send_zero_velocity(self, duration=2.0):
        """发送零速度命令，持续指定时间（秒）"""
        twist = Twist()
        # 发送多次以确保机器人停止
        rate = self.create_rate(10)  # 10Hz
        for _ in range(int(duration * 10)):
            self.velocity_pub.publish(twist)
            rate.sleep()

    def getKey(self):
        while True:
            char = getch.getch()
            print(f"你按下了: {char}")
            if char =='q':
                break
            elif char == 'a':
                # self.pause_navigation()
                self.cancel_all_goals()
            elif char == 'z':
                self.cancel_goal()
            elif char == 's':
                self.resume_navigation()
        # 这里可以添加其他不依赖键盘输入的代码逻辑

    def status_callback(self, msg: GoalStatusArray):
        # 检查是否有活动的导航目标
        if len(msg.status_list) > 0:
            # 获取最新的目标状态
            latest_status = msg.status_list[-1]
            
            # 状态码参考:
            # 1: STATUS_ACCEPTED (已接受)
            # 2: STATUS_EXECUTING (执行中)
            # 3: STATUS_CANCELING (取消中)
            # 4: STATUS_SUCCEEDED (已成功)
            # 5: STATUS_CANCELED (已取消)
            # 6: STATUS_ABORTED (已中止)
            
            self.is_navigating = latest_status.status in [
                GoalStatus.STATUS_ACCEPTED,
                GoalStatus.STATUS_EXECUTING,
                GoalStatus.STATUS_CANCELING
            ]
            
            status_text = {
                GoalStatus.STATUS_ACCEPTED: "已接受",
                GoalStatus.STATUS_EXECUTING: "执行中",
                GoalStatus.STATUS_CANCELING: "取消中",
                GoalStatus.STATUS_SUCCEEDED: "已成功",
                GoalStatus.STATUS_CANCELED: "已取消",
                GoalStatus.STATUS_ABORTED: "已中止"
            }.get(latest_status.status, f"未知状态({latest_status.status})")
            
            # self.get_logger().info(f"导航状态: {status_text}, 是否正在导航: {self.is_navigating}")
        else:
            pass
            # self.is_navigating = False
            # self.get_logger().info("没有活动的导航目标")

    def cancel_all_goals(self):
        """取消所有导航目标"""
        if not self.navigate_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('导航服务器不可用')
            return
        
        print("self.navigate_client:",self.navigate_client)
        # 获取当前GoalHandle（如果有）
        if hasattr(self.navigate_client, '_goal_handle') and self.navigate_client._goal_handle:
            # 取消当前活跃目标
            cancel_future = self.navigate_client._goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel_future)
            
            if cancel_future.result().return_code == cancel_future.result().SUCCESSFUL:
                self.get_logger().info('导航目标已成功取消')
            else:
                self.get_logger().warn('取消导航目标失败')
        else:
            self.get_logger().info('没有活跃的导航目标')


    def cancel_goal(self):
        # if not self.navigate_client.wait_for_server(timeout_sec=2.0):
        #     self.get_logger().warn('动作服务器不可用')
        #     return

        # # 发送取消指令
        # cancel_future = self.navigate_client.cancel_goal_async()
        # rclpy.spin_until_future_complete(self, cancel_future)

        # if cancel_future.result():
        #     self.get_logger().info('导航任务已取消')
        # else:
        #     self.get_logger().warn('取消导航任务失败')
         
        if not self.navigate_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('导航服务器不可用')
            return
            
        # 创建空的GoalID消息（使用字典替代）
        cancel_msg = {
            'goal_id': {
                'uuid': [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]  # 空UUID
            }
        }
        
        # 调用底层取消方法
        cancel_future = self.navigate_client._action_client._cancel_goal(cancel_msg)
        rclpy.spin_until_future_complete(self, cancel_future)
        
        if cancel_future.result():
            canceled_goals = len(cancel_future.result().goals_canceling)
            self.get_logger().info(f"已取消 {canceled_goals} 个导航目标")
        else:
            self.get_logger().error("取消请求失败")

def pause_navigation():
    rclpy.init()
    controller = NavigationController()
    
    try:
        pass
        # controller.pause_navigation()
        rclpy.spin(controller)
    except Exception as e:
        controller.get_logger().error(f'暂停导航时出错: {str(e)}')
    finally:
        controller.destroy_node()
        rclpy.shutdown()

def resume_navigation(target_pose):
    rclpy.init()
    controller = NavigationController()
    
    try:
        success = controller.resume_navigation(target_pose)
        return success
    except Exception as e:
        controller.get_logger().error(f'恢复导航时出错: {str(e)}')
        return False
    finally:
        controller.destroy_node()
        rclpy.shutdown()

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
    # # 1. 使用argparse解析自定义参数
    # parser = argparse.ArgumentParser()
    # parser.add_argument("-o", "--operation", type=str, default="pause", help="操作类型: pause/resume")
    # parser.add_argument("-x", type=float, default=0.0, help="目标X坐标")
    # parser.add_argument("-y", type=float, default=0.0, help="目标Y坐标")
    
    # # 分离ROS参数和自定义参数
    # ros_args = rclpy.utilities.remove_ros_args(args)
    # custom_args = parser.parse_args(ros_args[1:])  # 跳过程序名
    
    # # 根据参数执行逻辑
    # if custom_args.operation == "pause":
    #     pause_navigation()
    #     # 暂停导航
    #     pass
    # elif custom_args.operation == "resume":
    #     # 恢复导航到 (custom_args.x, custom_args.y)
    #     pass
    rclpy.init()
    controller = NavigationController()
    
    try:
        pass
        # controller.pause_navigation()
        rclpy.spin(controller)
    except Exception as e:
        controller.get_logger().error(f'暂停导航时出错: {str(e)}')
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()