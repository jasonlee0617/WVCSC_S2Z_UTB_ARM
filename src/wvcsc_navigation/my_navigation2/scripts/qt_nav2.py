#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient, ActionServer, GoalResponse, CancelResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

# ROS 2 Navigation2 相关消息/动作导入
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose  # 替换 move_base_msgs -> nav2_msgs
from action_msgs.msg import GoalStatusArray, GoalStatus
from action_msgs.srv import CancelGoal
from std_srvs.srv import SetBool
from unique_identifier_msgs.msg import UUID
import uuid

'''
* Author: zcj
* Alter: zcj
* Class:  NavigationGateway
* Description: 导航网关使能类（ROS 2 Navigation2 版本）
'''
class NavigationGateway(Node):
    def __init__(self):
        super().__init__('navigation_gateway')
        
        # 回调组（处理多线程/并发回调）
        self.callback_group = ReentrantCallbackGroup()
        
        # 导航使能状态（默认开启，可根据需求调整）
        self.enabled = True
        
        # 1. 处理Rviz的/move_base_simple/goal话题（兼容Rviz默认发布话题）
        self.simple_goal_pub = self.create_publisher(
            PoseStamped,
            '/filtered_move_base_simple/goal',
            10
        )
        self.simple_goal_sub = self.create_subscription(
            PoseStamped,
            '/move_base_simple/goal',
            self.handle_simple_goal,
            10,
            callback_group=self.callback_group
        )

        # 2. 订阅导航取消指令话题（ROS 2 标准取消话题）
        self.cancel_sub = self.create_subscription(
            GoalStatusArray,
            '/move_base/cancel_goal',
            self.handle_cancel,
            10,
            callback_group=self.callback_group
        )
        
        # 3. 连接到实际的 Navigation2 动作服务（NavigateToPose）
        self.nav2_client = ActionClient(
            self,
            NavigateToPose,
            '/internal_move_base',  # 实际导航节点的Action名称（通常为 /navigate_to_pose）
            callback_group=self.callback_group
        )
        self.get_logger().info("等待连接到Navigation2服务器...")
        while not self.nav2_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("Navigation2服务器未连接，重试中...")
        self.get_logger().info("Navigation2服务器连接成功")
        
        # 4. 创建对外的动作服务（标准 /move_base 名称，兼容原有调用逻辑）
        self.action_server = ActionServer(
            self,
            NavigateToPose,
            '/move_base',
            execute_callback=self.handle_action_goal,
            goal_callback=self.handle_goal_request,
            cancel_callback=self.handle_cancel_request,
            callback_group=self.callback_group
        )
        
        # 5. 使能控制服务
        self.enable_service = self.create_service(
            SetBool,
            '/navigation/enable',
            self.handle_enable_request,
            callback_group=self.callback_group
        )
        
        # 6. 导航状态缓存 + 状态订阅
        self.nav_status = None
        self.status_sub = self.create_subscription(
            GoalStatusArray,
            '/internal_move_base/status',  # 实际导航节点的状态话题
            self.update_nav_status,
            10,
            callback_group=self.callback_group
        )

        # 7. ROS 2 取消目标的服务客户端
        self.cancel_goal_client = self.create_client(
            CancelGoal,
            '/internal_move_base/cancel_goal',
            callback_group=self.callback_group
        )
        while not self.cancel_goal_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("cancel_goal服务未连接，重试中...")

        # 初始化日志
        if self.enabled:
            self.get_logger().info("导航全局网关已启动，初始状态：启用")
        else:
            self.get_logger().info("导航全局网关已启动，初始状态：禁用")

    '''
    * 处理使能状态切换
    '''
    def handle_enable_request(self, req, res):
        self.enabled = req.data
        res.success = True
        res.message = f"导航已{'启用' if self.enabled else '禁用'}"
        self.get_logger().info(res.message)
        if not self.enabled:
            # 异步取消导航（需封装为任务）
            rclpy.task.Future().add_done_callback(lambda _: self.cancel_navigation())
        return res

    '''
    * 处理Rviz的简单目标转发
    '''
    def handle_simple_goal(self, msg):
        if not self.enabled:
            self.get_logger().warn("Rviz导航被拒绝：当前未使能")
            return
        self.get_logger().info("转发Rviz导航目标到过滤话题")
        self.simple_goal_pub.publish(msg)

    '''
    * Action目标请求预处理
    '''
    def handle_goal_request(self, goal_request):
        if not self.enabled:
            self.get_logger().warn("动作导航被拒绝：当前未使能")
            return GoalResponse.REJECT
        self.get_logger().info("接受导航目标请求")
        return GoalResponse.ACCEPT

    '''
    * Action取消请求预处理
    '''
    def handle_cancel_request(self, cancel_request):
        self.get_logger().info("接受导航取消请求")
        return CancelResponse.ACCEPT

    '''
    * 处理导航Action目标执行（核心逻辑）
    '''
    async def handle_action_goal(self, goal_handle: ServerGoalHandle):
        self.get_logger().info("转发导航目标到Navigation2节点")
        
        # 构建Navigation2目标（适配NavigateToPose结构）
        nav2_goal = NavigateToPose.Goal()
        nav2_goal.pose = goal_handle.request.pose  # 关键：MoveBase.target_pose → NavigateToPose.pose
        nav2_goal.behavior_tree = ""  # 可选：指定行为树（默认空）
        
        # 发送目标到实际Navigation2节点
        send_goal_future = self.nav2_client.send_goal_async(
            nav2_goal,
            feedback_callback=self.handle_nav2_feedback
        )
        send_goal_future = await send_goal_future
        
        if not send_goal_future.accepted:
            self.get_logger().error("Navigation2拒绝接收目标")
            goal_handle.abort(NavigateToPose.Result(), "Navigation2拒绝接收目标")
            return
        
        # 等待目标执行结果
        result_future = send_goal_future.get_result_async()
        result_future = await result_future
        
        # 处理执行结果
        result = NavigateToPose.Result()
        if result_future.result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("导航任务执行成功")
            goal_handle.succeed(result)
        else:
            self.get_logger().warn(f"导航任务执行失败，状态码：{result_future.result.status}")
            goal_handle.abort(result, f"导航失败，状态码：{result_future.result.status}")

    '''
    * 转发Navigation2反馈给请求方
    '''
    def handle_nav2_feedback(self, feedback_msg):
        """处理Navigation2的反馈并转发"""
        feedback = NavigateToPose.Feedback()
        feedback.current_pose = feedback_msg.feedback.current_pose  # 关键：base_position → current_pose
        feedback.distance_remaining = feedback_msg.feedback.distance_remaining
        feedback.time_remaining = feedback_msg.feedback.time_remaining
        # 若需主动发布反馈，需缓存goal_handle并调用：
        # if self.current_goal_handle:
        #     self.current_goal_handle.publish_feedback(feedback)
        self.get_logger().debug(f"导航反馈：剩余距离 {feedback.distance_remaining:.2f}m")

    '''
    * 取消当前导航任务（ROS 2 异步实现）
    '''
    async def cancel_navigation(self):
        try:
            if not self.is_navigation_active():
                self.get_logger().info("当前无活跃导航任务，无需取消")
                return
            
            self.get_logger().info("开始取消导航任务")
            
            # 构建取消请求（取消所有目标）
            cancel_req = CancelGoal.Request()
            cancel_req.goal_info.goal_id.uuid = UUID(uuid=bytes([0]*16))  # 空UUID表示取消全部
            
            # 发送取消请求
            cancel_future = self.cancel_goal_client.call_async(cancel_req)
            cancel_resp = await cancel_future
            
            if cancel_resp.success:
                self.get_logger().info("导航任务取消成功")
            else:
                self.get_logger().error(f"导航任务取消失败：{cancel_resp.error_message}")
                
        except Exception as e:
            self.get_logger().error(f"取消导航异常: {str(e)}")

    '''
    * 更新导航状态缓存
    '''
    def update_nav_status(self, msg):
        self.nav_status = msg

    '''
    * 判断导航是否活跃
    '''
    def is_navigation_active(self):
        if self.nav_status is None:
            return False
        for status in self.nav_status.status_list:
            # ROS 2 活跃状态：Pending(1) / Executing(2)
            if status.status in [GoalStatus.STATUS_PENDING, GoalStatus.STATUS_EXECUTING]:
                return True
        return False

    '''
    * 处理外部取消指令
    '''
    async def handle_cancel(self, msg):
        self.get_logger().info("收到外部取消导航指令")
        await self.cancel_navigation()

'''
* 主函数
'''
def main(args=None):
    # 初始化ROS 2
    rclpy.init(args=args)
    
    # 创建节点
    gateway = NavigationGateway()
    
    # 多线程执行器（必须，处理异步Action回调）
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(gateway)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        gateway.get_logger().info("节点被手动中断")
    except Exception as e:
        gateway.get_logger().error(f"节点运行异常：{str(e)}")
    finally:
        # 资源清理
        gateway.action_server.destroy()
        gateway.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()