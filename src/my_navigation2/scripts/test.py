#!/usr/bin/env python3
import sys
import os
import json
import rclpy
from rclpy.node import Node
import uuid
from threading import Timer
from PyQt5.QtWidgets import (QApplication, QWidget, QPushButton, 
                            QVBoxLayout, QHBoxLayout, QLabel, QTextEdit,
                            QMessageBox, QFileDialog)
from PyQt5.QtCore import Qt, QThread, pyqtSignal
from PyQt5.QtCore import QTimer  # 导入QTimer
from PyQt5.QtCore import QTimer
import threading

# 定义保存点的默认文件路径
DEFAULT_SAVE_PATH = os.path.expanduser("~/navigation_points.json")

# '''
# * Author: zcj
# * Alter: zcj
# * Class:  NavigationThread
# * Description: 导航线程类
# '''
# class NavigationThread(QThread):
#     """导航线程，避免阻塞UI"""
#     navigation_complete = pyqtSignal(bool, str)  # 导航完成信号 (成功, 消息)
#     '''
#     * Author: zcj
#     * Alter: zcj
#     * Function:  __init__
#     * Description: 初始化函数
#     * Input:无
#     * Output:
#     *   goal_pose：目标位置 
#     * Return:无 
#     '''
#     def __init__(self, goal_pose):
#         try:
#             super(NavigationThread, self).__init__()
#             self.goal_pose = goal_pose
#             self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
#             self.running = True

#             # 定义所有“完成状态”的集合
#             self.DONE_STATES = [
#                 GoalStatus.SUCCEEDED,    # 3: 成功完成
#                 GoalStatus.ABORTED,      # 4: 异常终止
#                 GoalStatus.PREEMPTED,    # 2: 被抢占
#                 GoalStatus.REJECTED,     # 5: 被拒绝
#                 GoalStatus.RECALLED,     # 8: 被撤回
#                 GoalStatus.LOST          # 9: 丢失
#             ]
#         except Exception as e:
#             print("NavigationThread init error :",e)
    
#     '''
#     * Author: zcj
#     * Alter: zcj
#     * Function: run 
#     * Description: 线程要运行的函数
#     * Input:无
#     * Output:无
#     * Return:无 
#     '''
#     def run(self):
#         try:
#             # 等待move_base服务器
#             if not self.client.wait_for_server(rospy.Duration(5.0)):
#                 self.navigation_complete.emit(False, "无法连接到move_base服务器")
#                 return
#             # 创建导航目标
#             goal = MoveBaseGoal()
#             goal.target_pose.header.frame_id = "map"
#             goal.target_pose.header.stamp = rospy.Time.now()
#             goal.target_pose.pose = self.goal_pose
            
#             # 发送目标
#             print("发送目标")
#             self.client.send_goal(goal)
            
#             # 等待结果（关键修改：循环仅在客户端未进入DONE状态时运行）
#             while self.running and  self.client.get_state() not in self.DONE_STATES:
#                 # 等待0.5秒，超时则继续循环
#                 if self.client.wait_for_result(rospy.Duration(0.5)):
#                     break  # 目标完成，退出循环
#                 if rospy.is_shutdown():
#                     self.navigation_complete.emit(False, "ROS节点已关闭")
#                     self.running = False
#                     return
                    
#             # 循环退出后，强制标记为非运行状态
#             self.running = False 

#             # 仅在客户端确实进入DONE状态后才检查结果
#             if self.client.get_state() in self.DONE_STATES:
#                 state = self.client.get_state()
#                 if state == actionlib.GoalStatus.SUCCEEDED:
#                     self.navigation_complete.emit(True, "导航成功到达目标点")
#                 else:
#                     # ... 错误处理 ...
#                     err = ''
#                     if state == 2:
#                         err = '目标被客户端主动取消'
#                     elif state == 4:
#                         err = '无法导航或取消导航'
#                     elif state == 9:
#                         err = '目标状态未知'
#                     self.navigation_complete.emit(False, f"导航失败，状态码: {state}。{err}")
#             else:
#                 # 未进入DONE状态（如被外部取消）
#                 self.navigation_complete.emit(False, "导航已取消")
#         except Exception as e:
#             print("NavigationThread run error :",e)

#     '''
#     * Author: zcj
#     * Alter: zcj
#     * Function: stop
#     * Description: 停止线程，停止导航
#     * Input:无
#     * Output:无
#     * Return:无 
#     '''    
#     def stop(self):
#         try:
#             """停止导航"""
#             self.running = False
#             if hasattr(self, 'client') and self.client is not None:
#                 if self.client.get_state() in [actionlib.GoalStatus.ACTIVE, 
#                                             actionlib.GoalStatus.PENDING]:
#                     self.client.cancel_goal()
#             self.wait()
#         except Exception as e:
#             print("NavigationThread stop error :",e)

'''
* class:   NavigatorUI
* Description: 带QT界面的navigation规划类
'''
class NavigatorUI(QWidget):
    '''
    * Author: zcj
    * Alter: zcj
    * Function:  __init__
    * Description: 初始化函数
    * Input: 无
    * Output:无 
    * Return:无 
    '''
    def __init__(self):
        super().__init__()
        
        # 存储目标点及标记ID
        self.point1 = None
        self.point2 = None
        self.marker_ids = {}  # 存储点的标记ID: {1: id, 2: id}
        self.current_phase = 0  # 0: 未开始, 1: 已到第一点, 2: 已到第二点
        self.nav_thread = None
        self.save_path = DEFAULT_SAVE_PATH
        
        # 初始化ROS节点
        # rclpy.init_node('qt_ros_navigator', anonymous=True)
        # 设置UI
        self.ui_init()

        # 订阅当前位置
        # self.pose_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, 
                                        # self.update_current_pose)
        self.current_pose = None

        # 创建标记数组发布器
        # self.marker_array_pub = rospy.Publisher('/navigation_markers', MarkerArray, queue_size=10)
        
        # 标记发布间隔（秒）
        self.marker_publish_interval = 1.0  # 每秒发布一次
        # self.start_marker_publishing()

        self.marker_timer = QTimer(self)  # 创建QTimer
        self.marker_timer.timeout.connect(self.publish_marker_array)  # 绑定发布函数
        self.marker_timer.start(int(self.marker_publish_interval * 1000))  # 单位：毫秒
        
        self.nav_status = None  # 缓存导航状态
        # self.status_sub = rospy.Subscriber('/internal_move_base/status', GoalStatusArray, self.update_nav_status)

        # self.cancel_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=10)  # 初始化一次
        # self.cancel_pub = rospy.Publisher('/internal_move_base/cancel', GoalID, queue_size=10)  # 初始化一次
        # 尝试加载之前保存的点
        self.load_points()

        self.mqtt_init()

 
    '''
    * Author: zcj
    * Alter: zcj
    * Function:  ui_init
    * Description:UI 初始化函数
    * Input: 无
    * Output:无 
    * Return:无 
    '''    
    def ui_init(self):
        try:
            # 设置窗口标题和大小
            self.setWindowTitle('ROS导航控制器')
            self.setGeometry(300, 300, 600, 450)
            
            # 创建按钮
            self.btn_record1 = QPushButton('记录起点')
            self.btn_record2 = QPushButton('记录终点')
            self.btn_navigate1 = QPushButton('开始导航1')
            self.btn_navigate2 = QPushButton('开始导航2')
            self.btn_save = QPushButton('保存点到文件')
            self.btn_load = QPushButton('从文件加载点')

            self.btn_cancel = QPushButton('取消导航')  # 新增取消按钮
            
            # 设置按钮大小
            self.btn_record1.setMinimumHeight(50)
            self.btn_record2.setMinimumHeight(50)
            self.btn_navigate1.setMinimumHeight(50)
            self.btn_navigate2.setMinimumHeight(50)
            self.btn_save.setMinimumHeight(30)
            self.btn_load.setMinimumHeight(30)

            self.btn_cancel.setMinimumHeight(50)  # 取消按钮大小
            
            # 连接按钮信号
            self.btn_record1.clicked.connect(self.record_point1)
            self.btn_record2.clicked.connect(self.record_point2)
            self.btn_navigate1.clicked.connect(self.start_navigation1)
            self.btn_navigate2.clicked.connect(self.start_navigation2)
            self.btn_save.clicked.connect(self.save_points_dialog)
            self.btn_load.clicked.connect(self.load_points_dialog)

            self.btn_cancel.clicked.connect(self.cancel_navigation)  # 绑定取消导航函数
            
            # 创建状态显示区域
            self.status_label = QLabel('状态: 等待初始化...')
            self.status_label.setAlignment(Qt.AlignCenter)
            self.status_label.setStyleSheet("font-size: 14px; color: blue;")
            
            self.log_area = QTextEdit()
            self.log_area.setReadOnly(True)
            self.log_area.setStyleSheet("font-family: monospace;")
            
            # 布局管理
            btn_layout = QHBoxLayout()
            btn_layout.addWidget(self.btn_record1)
            btn_layout.addWidget(self.btn_record2)
            btn_layout.addWidget(self.btn_navigate1)
            btn_layout.addWidget(self.btn_navigate2)

            btn_layout.addWidget(self.btn_cancel)  # 添加取消按钮
            
            file_btn_layout = QHBoxLayout()
            file_btn_layout.addWidget(self.btn_save)
            file_btn_layout.addWidget(self.btn_load)
            
            main_layout = QVBoxLayout()
            main_layout.addLayout(btn_layout)
            main_layout.addLayout(file_btn_layout)
            main_layout.addWidget(self.status_label)
            main_layout.addWidget(self.log_area)
            
            self.setLayout(main_layout)
            
            # 显示初始信息
            self.update_status("就绪，请记录目标点")
            self.add_log("程序已启动，等待接收车辆位置...")
            
            # 显示已加载的点信息
            if self.point1 is not None:
                pos = self.point1.position
                self.add_log(f"已加载起点 - x: {pos.x:.2f}, y: {pos.y:.2f}, z: {pos.z:.2f}")
            if self.point2 is not None:
                pos = self.point2.position
                self.add_log(f"已加载终点 - x: {pos.x:.2f}, y: {pos.y:.2f}, z: {pos.z:.2f}")
        except Exception as e:
            print('ui_init error:',e)

    '''
    * Author: zcj
    * Alter: zcj
    * Function:  create_marker
    * Description: 创建标记
    * Input: 
    *       pose：标记位置
    *       point_id：标记ID
    * Output:无 
    * Return:无 
    '''    
    def create_marker(self, pose, point_id):
        """创建标记"""
        try:
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rclpy.Time.now()
            marker.ns = "navigation_points"
            marker.id = self.marker_ids.get(point_id, 0)
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            # 设置标记位置和大小
            marker.pose = pose
            marker.scale.x = 0.4  # 球体直径
            marker.scale.y = 0.4
            marker.scale.z = 0.4
            
            # 设置颜色
            marker.color.a = 0.8  # 透明度
            
            # 根据点ID和导航阶段设置颜色
            if point_id == 1:
                if self.current_phase >= 1:  # 已到达第一点
                    marker.color.r = 1.0  # 红色
                    marker.color.g = 0.0
                    marker.color.b = 0.0
                else:  # 未到达
                    marker.color.r = 0.0
                    marker.color.g = 1.0  # 绿色
                    marker.color.b = 0.0
            else:  # point_id == 2
                if self.current_phase >= 2:  # 已到达第二点
                    marker.color.r = 1.0  # 红色
                    marker.color.g = 0.0
                    marker.color.b = 0.0
                elif self.current_phase == 1:  # 已到达第一点，未到达第二点
                    marker.color.r = 0.0
                    marker.color.g = 1.0  # 绿色
                    marker.color.b = 0.0
                else:  # 未到达第一点
                    marker.color.r = 0.0
                    marker.color.g = 0.0
                    marker.color.b = 1.0  # 蓝色
                    
            # 标记的生命周期（稍长于发布间隔）
            marker.lifetime = rclpy.Duration(2 * self.marker_publish_interval)
            
            return marker
        except Exception as e:
            print("create marker error:",e)
    
    '''
    * Author: zcj
    * Alter: zcj
    * Function:  publish_marker_array
    * Description: 发布标记数组
    * Input:无
    * Output:无 
    * Return:无 
    '''    
    def publish_marker_array(self):
        try:
            """发布标记数组"""
            if rclpy.is_shutdown():
                return
            marker_array = MarkerArray()
                
            # 添加起点的标记
            if self.point1 is not None:
                if 1 not in self.marker_ids:
                    self.marker_ids[1] = uuid.uuid4().int % 100000
                marker = self.create_marker(self.point1, 1)
                marker_array.markers.append(marker)
                
            # 添加终点的标记
            if self.point2 is not None:
                if 2 not in self.marker_ids:
                    self.marker_ids[2] = uuid.uuid4().int % 100000
                marker = self.create_marker(self.point2, 2)
                marker_array.markers.append(marker)
                
            self.marker_array_pub.publish(marker_array)
        except Exception as e:
            print("publish_marker_array error:",e)

    '''
    * Author: zcj
    * Alter: zcj
    * Function:  start_marker_publishing
    * Description: 开始定期发布标记数组
    * Input:无
    * Output:无 
    * Return:无 
    '''    
    def start_marker_publishing(self):
        try:
            self.publish_marker_array()
        except Exception as e:
            print("start_marker_publishing error:",e)
       
    
    '''
    * Author: zcj
    * Alter: zcj
    * Function:  update_current_pose
    * Description: 更新当前位置
    * Input: msg：当前位置消息
    * Output:无 
    * Return:无 
    '''    
    def update_current_pose(self, msg):
        try:
            self.current_pose = msg.pose.pose
            # print("msg:",msg)
            if self.status_label.text().startswith('状态: 等待初始化'):
                self.update_status("就绪，请记录目标点")
                self.add_log("已接收到车辆位置信息")
        except Exception as e:
            print('update_current_pose error:',e)
        
    '''
    * Author: zcj
    * Alter: zcj
    * Function:  record_point1
    * Description: 记录起点
    * Input:无
    * Output:无 
    * Return:无 
    '''    
    def record_point1(self):
        """记录起点"""
        # 如果已有保存的点，询问是否覆盖
        if self.point1 is not None:
            reply = QMessageBox.question(self, '确认覆盖', 
                                        '已有起点记录，是否覆盖?',
                                        QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
            if reply == QMessageBox.No:
                return
                
        if self.current_pose is None:
            self.update_status("错误: 未获取到车辆位置")
            self.add_log("无法记录起点: 未获取到车辆位置")
            return
            
        self.point1 = self.current_pose
        # 生成唯一ID
        self.marker_ids[1] = uuid.uuid4().int % 100000
        self.update_status("已记录起点")
        pos = self.point1.position
        self.add_log(f"已记录起点 - x: {pos.x:.2f}, y: {pos.y:.2f}, z: {pos.z:.2f}")
        # 自动保存
        self.save_points()

    '''
    * Author: zcj
    * Alter: zcj
    * Function:  record_point2
    * Description: 记录终点
    * Input:无
    * Output:无 
    * Return:无 
    '''    
    def record_point2(self):
        """记录终点"""
        # 如果已有保存的点，询问是否覆盖
        if self.point2 is not None:
            reply = QMessageBox.question(self, '确认覆盖', 
                                        '已有终点记录，是否覆盖?',
                                        QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
            if reply == QMessageBox.No:
                return
                
        if self.current_pose is None:
            self.update_status("错误: 未获取到车辆位置")
            self.add_log("无法记录终点: 未获取到车辆位置")
            return
            
        self.point2 = self.current_pose
        # 生成唯一ID
        self.marker_ids[2] = uuid.uuid4().int % 100000
        self.update_status("已记录终点")
        pos = self.point2.position
        self.add_log(f"已记录终点 - x: {pos.x:.2f}, y: {pos.y:.2f}, z: {pos.z:.2f}")
        # 自动保存
        self.save_points()
    
    '''
    * Author: zcj
    * Alter: zcj
    * Function:  start_navigation1
    * Description: 开始导航到起点
    * Input:无
    * Output:无 
    * Return:无 
    '''    
    def start_navigation1(self):
        try:
            """开始导航"""
            # 检查是否正在导航
            if self.nav_thread and self.nav_thread.isRunning():
                self.update_status("正在导航中，请勿重复导航")
                return
                
            # 第一阶段：导航到起点
            if self.point1 is None:
                self.update_status("错误: 未记录起点")
                self.add_log("无法开始导航: 请先记录起点")
                return
            
            # self.current_phase = 1
            # self.update_status("正在导航到起点...")
            # self.add_log("开始导航到起点")
            # self.nav_thread = NavigationThread(self.point1)
            # self.nav_thread.navigation_complete.connect(self.on_nav_complete)
            # self.nav_thread.start()
        except Exception as e:
            print("start_navigation1 error:",e)
    
    '''
    * Author: zcj
    * Alter: zcj
    * Function:  start_navigation2
    * Description: 开始导航到终点
    * Input:无
    * Output:无 
    * Return:无 
    '''    
    def start_navigation2(self):
        try:
            # 检查是否正在导航
            if self.nav_thread and self.nav_thread.isRunning():
                self.update_status("正在导航中，请勿重复导航")
                return
                
            # 导航到终点
            if self.point2 is None:
                self.update_status("错误: 未记录终点")
                self.add_log("无法继续导航: 请先记录终点")
                return
            
            self.current_phase = 2
            self.update_status("正在导航到终点...")
            self.add_log("开始导航到终点")
            # self.nav_thread = NavigationThread(self.point2)
            # self.nav_thread.navigation_complete.connect(self.on_nav_complete)
            # self.nav_thread.start()
        except Exception as e:
            print("start_navigation2 error:",e)
    
    '''
    * Author: zcj
    * Alter: zcj
    * Function:  on_nav_complete
    * Description: 导航完成回调
    * Input: 
    *       success：是否成功
    *       message：导航结果
    * Output:无 
    * Return:无 
    '''    
    def on_nav_complete(self, success, message):
        try:
            self.add_log(f"导航结果: {message} ")
            
            if success:
                if self.current_phase == 1:
                    self.update_status("已到达起点")
                    
                    jsonData = {"destination":1}
                    jsonString = json.dumps(jsonData, indent=4)
                    self.my_mqtt.Publish(self.car_destination_state_Pub, jsonString)

                    if self.CarMode == "zkwl":
                        pass
                    elif self.CarMode == "wtb":
                        pass
                        
                elif self.current_phase == 2:
                    jsonData = {"destination":2}
                    jsonString = json.dumps(jsonData, indent=4)
                    self.my_mqtt.Publish(self.car_destination_state_Pub, jsonString)
                    self.update_status("已到达终点")
                    if self.CarMode == "zkwl":
                        pass
                        #开始伸出采摘框
                        self.SendFrameControl(1)
                    
            else:
                jsonData = {
                            "destination":0
                            }
                jsonString = json.dumps(jsonData, indent=4)
                self.my_mqtt.Publish(self.car_destination_state_Pub, jsonString)
                self.update_status(f"导航失败: {message}")

            # 停止线程并确保资源释放
            self.nav_thread.stop()
            # self.nav_thread.wait()  # 等待线程完全停止     
            self.nav_thread = None
        except Exception as e:
            print("on_nav_complete error:",e)
    
    '''
    * Author: zcj
    * Alter: zcj
    * Function:  update_status
    * Description: 更新状态标签
    * Input: 
    *       text：状态文本
    * Output:无 
    * Return:无 
    '''    
    def update_status(self, text):
        """更新状态标签"""
        self.status_label.setText(f"状态: {text}")

    '''
    * Author: zcj
    * Alter: zcj
    * Function:  add_log
    * Description: 添加日志信息
    * Input: 
    *       text：日志文本
    * Output:无 
    * Return:无 
    '''    
    def add_log(self, text):
        """添加日志信息"""
        timestamp = rclpy.get_time()
        # print(f"[{timestamp:.2f}] {text}")
        self.log_area.append(f"[{timestamp:.2f}] {text}")
        # 自动滚动到底部
        # self.log_area.moveCursor(self.log_area.textCursor().End)

    '''
    * Author: zcj
    * Alter: zcj
    * Function:  save_points
    * Description: 保存点到文件
    * Input: 
    *       path：文件路径
    * Output:无 
    * Return:无 
    '''    
    def save_points(self, path=None):
        """保存点到文件"""
        if path is None:
            path = self.save_path
            
        try:
            data = {}
            if self.point1 is not None:
                data['point1'] = {
                    'position': {
                        'x': self.point1.position.x,
                        'y': self.point1.position.y,
                        'z': self.point1.position.z
                    },
                    'orientation': {
                        'x': self.point1.orientation.x,
                        'y': self.point1.orientation.y,
                        'z': self.point1.orientation.z,
                        'w': self.point1.orientation.w
                    }
                }
                
            if self.point2 is not None:
                data['point2'] = {
                    'position': {
                        'x': self.point2.position.x,
                        'y': self.point2.position.y,
                        'z': self.point2.position.z
                    },
                    'orientation': {
                        'x': self.point2.orientation.x,
                        'y': self.point2.orientation.y,
                        'z': self.point2.orientation.z,
                        'w': self.point2.orientation.w
                    }
                }
                
            with open(path, 'w') as f:
                json.dump(data, f, indent=4)
                
            self.save_path = path
            self.add_log(f"已保存点到文件: {path}")
            return True
            
        except Exception as e:
            self.add_log(f"保存点失败: {str(e)}")
            QMessageBox.warning(self, "保存失败", f"无法保存点到文件: {str(e)}")
            return False
    
    '''
    * Author: zcj
    * Alter: zcj
    * Function:  save_points_dialog
    * Description: 打开对话框选择保存路径
    * Input:无
    * Output:无 
    * Return:无 
    '''    
    def save_points_dialog(self):
        """打开对话框选择保存路径"""
        if self.point1 is None and self.point2 is None:
            QMessageBox.information(self, "无数据", "没有可保存的点，请先记录点")
            return
            
        path, _ = QFileDialog.getSaveFileName(
            self, "保存导航点", 
            os.path.expanduser("~"), 
            "JSON文件 (*.json)"
        )
        
        if path:
            self.save_points(path)
    
    '''
    * Author: zcj
    * Alter: zcj
    * Function:  load_points
    * Description: 从文件加载点
    * Input: 
    *       path：文件路径
    * Output:无 
    * Return:无 
    '''    
    def load_points(self, path=None):
        """从文件加载点"""
        if path is None:
            path = self.save_path
            # 检查默认文件是否存在
            if not os.path.exists(path):
                return False
                
        try:
            with open(path, 'r') as f:
                data = json.load(f)
                
            # 加载起点
            if 'point1' in data:
                from geometry_msgs.msg import Pose, Point, Quaternion
                point_data = data['point1']
                pose = Pose()
                pose.position = Point(
                    x=point_data['position']['x'],
                    y=point_data['position']['y'],
                    z=point_data['position']['z']
                )
                pose.orientation = Quaternion(
                    x=point_data['orientation']['x'],
                    y=point_data['orientation']['y'],
                    z=point_data['orientation']['z'],
                    w=point_data['orientation']['w']
                )
                self.point1 = pose
                self.marker_ids[1] = uuid.uuid4().int % 100000
                
            # 加载终点
            if 'point2' in data:
                from geometry_msgs.msg import Pose, Point, Quaternion
                point_data = data['point2']
                pose = Pose()
                pose.position = Point(
                    x=point_data['position']['x'],
                    y=point_data['position']['y'],
                    z=point_data['position']['z']
                )
                pose.orientation = Quaternion(
                    x=point_data['orientation']['x'],
                    y=point_data['orientation']['y'],
                    z=point_data['orientation']['z'],
                    w=point_data['orientation']['w']
                )
                self.point2 = pose
                self.marker_ids[2] = uuid.uuid4().int % 100000
                
            self.save_path = path
            self.add_log(f"已从文件加载点: {path}")
            return True
            
        except Exception as e:
            self.add_log(f"加载点失败: {str(e)}")
            return False
    
    '''
    * Author: zcj
    * Alter: zcj
    * Function:  load_points_dialog
    * Description: 打开对话框选择加载路径
    * Input:无
    * Output:无 
    * Return:无 
    '''    
    def load_points_dialog(self):
        """打开对话框选择加载路径"""
        path, _ = QFileDialog.getOpenFileName(
            self, "加载导航点", 
            os.path.expanduser("~"), 
            "JSON文件 (*.json)"
        )
        
        if path:
            # 询问是否覆盖现有点
            if (self.point1 is not None or self.point2 is not None):
                reply = QMessageBox.question(self, '确认覆盖', 
                                            '现有记录的点将被覆盖，是否继续?',
                                            QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
                if reply == QMessageBox.No:
                    return
                    
            self.load_points(path)
    
    '''
    * Author: zcj
    * Alter: zcj
    * Function:  clear_markers
    * Description: 清除所有标记
    * Input:无
    * Output:无 
    * Return:无 
    '''    
    def clear_markers(self):
        """清除所有标记"""
        marker_array = MarkerArray()
        for marker_id in self.marker_ids.values():
            marker = Marker()
            marker.header.frame_id = "map"
            marker.ns = "navigation_points"
            marker.id = marker_id
            marker.action = Marker.DELETE
            marker_array.markers.append(marker)
        self.marker_array_pub.publish(marker_array)
        self.marker_ids = {}
    
    '''
    * Author: zcj
    * Alter: zcj
    * Function:  closeEvent
    * Description: 关闭窗口时的处理
    * Input:无
    * Output:无 
    * Return:无 
    '''    
    def closeEvent(self, event):
        """关闭窗口时的处理"""
        self.clear_markers()
        if hasattr(self, 'timer'):
            self.timer.cancel()
        if self.nav_thread and self.nav_thread.isRunning():
            self.nav_thread.stop()
        event.accept()

    '''
    * Author: zcj
    * Alter: zcj
    * Function:  cancel_navigation
    * Description: 取消当前正在执行的MoveBase导航任务
    * Input:无
    * Output:无 
    * Return:无 
    '''    
    def cancel_navigation(self):
        try:
            
            if not self.is_navigation_active():
                self.update_status("未在导航中，无需取消")
                if self.nav_thread and self.nav_thread.isRunning():
                    # 停止线程并确保资源释放
                    self.nav_thread.stop()
                    # self.nav_thread.wait()  # 等待线程完全停止     
                    self.nav_thread = None
                return
            print("取消导航")
            # 发布取消消息（使用已初始化的publisher）
            self.cancel_pub.publish(GoalID())  # 空消息取消所有目标
            rospy.sleep(0.1)  # 短暂延迟确保消息送达
            self.update_status("导航已取消")
        except Exception as e:
            rospy.logerr(f"取消导航错误: {e}")

    '''
    * Author: zcj
    * Alter: zcj
    * Function:  update_nav_status
    * Description: 实时更新导航状态
    * Input:无
    * Output:无 
    * Return:无 
    '''    
    def update_nav_status(self, msg):
        self.nav_status = msg  # 实时更新状态缓存

    '''
    * Author: zcj
    * Alter: zcj
    * Function:  is_navigation_active
    * Description: 通过缓存的状态判断导航状态
    * Input:无
    * Output:无 
    * Return:无 
    '''    
    def is_navigation_active(self):
        if self.nav_status is None:
            return False
        for status in self.nav_status.status_list:
            if status.status in [0, 1]:  # PENDING或ACTIVE
                return True
        return False

    '''
    * Author: zcj
    * Alter: zcj
    * Function:  mqtt_recv
    * Description: 接收MQTT消息的回调函数
    * Input:无
    * Output:无 
    * Return:无 
    '''    
    def mqtt_recv(self,topic,data):
        try:
            print("topic,data:",topic,data)
            #接收目的的话题
            if topic == self.car_destination_sub:
                if not isinstance(data, dict):
                    data = json.loads(data)      
                if "destination" in data:
                    destination = data['destination']
                    if destination == 1:
                        self.start_navigation1()
                    elif destination == 2:
                        self.start_navigation2()
            elif topic == "frame_state":
                if not isinstance(data, dict):
                    data = json.loads(data)      # 解析JSON
                if self.CarMode == "zkwl":
                    state = data.get("state", -1)  # 默认值为-1表示未知状态
                    if state == 0: #采摘框缩回
                        self.add_log("采摘框已缩回")
                        time.sleep(0.5)
                        self.start_navigation1()                 
                    elif state == 1: #采摘框伸出
                        pass
                elif self.CarMode == 'wtb':
                    state = data.get("state", -1)  # 默认值为-1表示未知状态
                    if state == 0: #采摘框缩回
                        pass
                        self.add_log("采摘框缩回完成")
       
                        # time.sleep(0.5)
                        # self.start_navigation1() 
                    elif state == 1: #采摘框伸出完成
                        # 创建定时器
                        threading.Timer(1, lambda: (
                            #开始机械臂工作 
                            self.SendArmControl(),
                        )).start() 
            elif topic == 'arm_state':
                state = data.get("state", -1)  # 默认值为-1表示未知状态
                
                if state == 0: #机械臂开始采摘
                    self.add_log("机械臂开始采摘")
                elif state == 1: #机械臂采摘完成
                    if self.CarMode == "zkwl":
                        self.add_log("采摘完成,缩回采摘框")
                        # 创建定时器
                        threading.Timer(2, lambda: (
                            #开始缩回采摘框
                            self.SendFrameControl(0),
                        )).start()                      
            elif topic == 'wtb_destination_state':
                if self.CarMode == "zkwl":
                    if not isinstance(data, dict):
                        data = json.loads(data)      # 解析JSON
                    if "destination" in data:
                        destination = data['destination']
                        if destination == 1: #到起点
                            # time.sleep(0.5)
                            # self.start_navigation1()  
                            pass
                        elif destination == 2: #到达目的地
                            print("采摘车已经到达，运输车准备出发")
                            time.sleep(0.5)
                            self.start_navigation2()
            elif topic == 'zkwl_destination_state':
                if self.CarMode == "wtb":
                    if not isinstance(data, dict):
                        data = json.loads(data)      # 解析JSON
                    if "destination" in data:
                        destination = data['destination']
                        if destination == 1: #到起点
                            time.sleep(0.5)
                            self.start_navigation1()  
                        
        except Exception as e:
            print("mqtt_recv error:",e) 

    '''
    * Author: zcj
    * Alter: zcj
    * Function:  SendArmControl
    * Description: 发送机械臂控制命令
    * Input:无
    * Output:无 
    * Return:无 
    '''
    def SendArmControl(self):
        try:
            payload = {"control": 1}  # 构造控制命令JSON
            # 发布控制命令到arm_control话题
            self.my_mqtt.Publish("arm_control", json.dumps(payload, ensure_ascii=False))
        except Exception as e:
            print('SendArmControl error:',e)

    '''
    * Author: zcj
    * Alter: zcj
    * Function:  SendFrameControl
    * Description: 发送采摘框控制命令
    * Input:
    *       cmd: 0 缩回 1 伸展
    * Output:无 
    * Return:无 
    '''
    def SendFrameControl(self,cmd):
        try:
            payload = {"control": cmd}  # 构造控制命令JSON
            # 发布控制命令到frame_control话题
            self.my_mqtt.Publish("frame_control", json.dumps(payload, ensure_ascii=False))
        except Exception as e:
            print("SendFrameControl error:",e)


        

if __name__ == '__main__':
    try:
        app = QApplication(sys.argv)
        window = NavigatorUI()
        window.show()
        sys.exit(app.exec_())
    except rospy.ROSInterruptException:
        print("程序被ROS中断")
    except Exception as e:
        print(f"发生错误: {str(e)}")
    