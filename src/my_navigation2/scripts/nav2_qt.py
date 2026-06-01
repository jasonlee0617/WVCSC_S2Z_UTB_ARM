#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from std_msgs.msg import Header
import numpy as np
from tf2_ros import TransformListener, Buffer, TransformException
from tf2_geometry_msgs import do_transform_pose

# Qt 导入
from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, 
                             QPushButton, QMessageBox, QLabel)
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPalette, QColor
from PyQt5.QtWidgets import (QApplication, QWidget, QPushButton, 
                            QVBoxLayout, QHBoxLayout, QLabel, QTextEdit,
                            QMessageBox, QFileDialog)
from rclpy.clock import Clock, ClockType
import datetime
import os

# 定义保存点的默认文件路径
DEFAULT_SAVE_PATH = os.path.expanduser("~/navigation_points.json")

# 导航客户端类
class Nav2Client(Node):
    def __init__(self):
        super().__init__('nav2_qt_client')
        
        # 创建导航动作客户端
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.get_logger().info("等待连接到Nav2服务器...")
        while not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn("Nav2服务器未连接，重试中...")
        self.get_logger().info("Nav2服务器连接成功")
        
        # 初始化TF2，与PathRecorder.py保持一致
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
     
        # 起点和终点
        self.start_pose = None
        self.end_pose = None

        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('base_link', 'base_footprint')

        self.frame_id = self.get_parameter('frame_id').value
        self.base_link = self.get_parameter('base_link').value

        self.feedback_show = None
        
    '''
    获取当前车辆位置
    '''
    def get_current_pose(self):
        # 获取当前位置
        try:
            transform = self.tf_buffer.lookup_transform(
                self.frame_id,
                self.base_link,
                rclpy.time.Time()
            )
            
            # 创建PoseStamped消息
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = self.frame_id
            pose_stamped.header.stamp = transform.header.stamp
            pose_stamped.pose.position.x = transform.transform.translation.x
            pose_stamped.pose.position.y = transform.transform.translation.y
            pose_stamped.pose.position.z = transform.transform.translation.z
            pose_stamped.pose.orientation = transform.transform.rotation
            
            self.get_logger().info(f"通过TF获取位置 (map->base_footprint)：x={pose_stamped.pose.position.x:.2f}, y={pose_stamped.pose.position.y:.2f}")
            return pose_stamped
        except TransformException as e:
            self.get_logger().warn(f"TF变换获取失败 (map->base_footprint): {e}")
            
           
    """记录当前位置为起点"""
    def record_start(self):
        
        current_pose = self.get_current_pose()
        if current_pose is None:
            return False, "未获取到当前位置"
        
        self.start_pose = current_pose
        self.get_logger().info(f"起点已记录：x={self.start_pose.pose.position.x:.2f}, y={self.start_pose.pose.position.y:.2f}")
        return True, "起点已记录"
    
    def record_end(self):
        """记录当前位置为终点"""
        current_pose = self.get_current_pose()
        if current_pose is None:
            return False, "未获取到当前位置"
        
        self.end_pose = current_pose
        self.get_logger().info(f"终点已记录：x={self.end_pose.pose.position.x:.2f}, y={self.end_pose.pose.position.y:.2f}")
        return True, "终点已记录"
    
    def navigate_to_pose(self, pose):
        """导航到指定位置"""
        if pose is None:
            return False, "目标位置未设置"
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        
        self.get_logger().info(f"发起导航到：x={pose.pose.position.x:.2f}, y={pose.pose.position.y:.2f}")
        
        # 发送导航目标
        send_goal_future = self.nav_client.send_goal_async(
            goal_msg, 
            feedback_callback=self.feedback_callback
        )
        
        return True, "导航已发起"
    
    def navigate_to_start(self):
        """导航到起点"""
        return self.navigate_to_pose(self.start_pose)
    
    def navigate_to_end(self):
        """导航到终点"""
        return self.navigate_to_pose(self.end_pose)
    
    def feedback_callback(self, feedback_msg):
        """处理导航反馈"""
        distance = feedback_msg.feedback.distance_remaining
        # self.get_logger().info(f"导航剩余距离：{distance:.2f}米")

        
        if self.feedback_show is not None:
            self.feedback_show(f"导航剩余距离：{distance:.2f}米")
        
        
    

# Qt主界面类
class Nav2GUI(QWidget):
    def __init__(self, nav_client):
        super().__init__()

        self.clock = Clock(clock_type=ClockType.ROS_TIME)

        self.save_path = DEFAULT_SAVE_PATH
        self.point1 = None
        self.point2 = None
        self.load_points()

        self.nav_client = nav_client
        self.nav_client.feedback_show = self.add_log
        # self.init_ui()
        self.init_ui2()

        
    def init_ui(self):
        """初始化UI界面"""
        # 设置窗口属性
        self.setWindowTitle('Nav2 导航控制器')
        # self.setGeometry(100, 100, 400, 300)
        self.setGeometry(300, 300, 600, 450)
        
        # 创建垂直布局
        layout = QVBoxLayout()
        
        # 设置样式
        self.setStyleSheet(""
            "QWidget {"
            "    background-color: #2c3e50;"
            "    color: #ecf0f1;"
            "    font-size: 14px;"
            "}"
            "QPushButton {"
            "    background-color: #3498db;"
            "    color: white;"
            "    border: none;"
            "    padding: 12px 20px;"
            "    margin: 8px;"
            "    border-radius: 4px;"
            "    font-size: 14px;"
            "}"
            "QPushButton:hover {"
            "    background-color: #2980b9;"
            "}"
            "QPushButton:pressed {"
            "    background-color: #1f618d;"
            "}"
            "QLabel {"
            "    color: #ecf0f1;"
            "    margin: 8px;"
            "    font-size: 14px;"
            "}"
        )
        
        # 添加状态标签
        self.status_label = QLabel("就绪")
        self.status_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.status_label)
        
        # 创建按钮
        self.record_start_btn = QPushButton('记录起点')
        self.record_end_btn = QPushButton('记录终点')
        self.go_start_btn = QPushButton('回到起点')
        self.go_end_btn = QPushButton('去到终点')
        
        # 连接按钮信号
        self.record_start_btn.clicked.connect(self.on_record_start)
        self.record_end_btn.clicked.connect(self.on_record_end)
        self.go_start_btn.clicked.connect(self.on_go_start)
        self.go_end_btn.clicked.connect(self.on_go_end)
        
        # 添加按钮到布局
        layout.addWidget(self.record_start_btn)
        layout.addWidget(self.record_end_btn)
        layout.addWidget(self.go_start_btn)
        layout.addWidget(self.go_end_btn)
        
        # 设置布局
        self.setLayout(layout)
    
    def init_ui2(self):
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


            
            # # 连接按钮信号
            self.btn_record1.clicked.connect(self.on_record_start) #记录起点
            self.btn_record2.clicked.connect(self.on_record_end) #记录终点
            self.btn_navigate1.clicked.connect(self.on_go_start)  #导航到起点
            self.btn_navigate2.clicked.connect(self.on_go_end) #导航到终点d
            self.btn_save.clicked.connect(self.save_points_dialog)
            self.btn_load.clicked.connect(self.load_points_dialog)
            # self.btn_cancel.clicked.connect(self.cancel_navigation)  # 绑定取消导航函数
            
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

    """记录起点按钮点击事件"""
    def on_record_start(self):
        success, message = self.nav_client.record_start()
        self.update_status(message)

    """记录终点按钮点击事件"""
    def on_record_end(self):
        success, message = self.nav_client.record_end()
        self.update_status(message)
    
    def on_go_start(self):
        """回到起点按钮点击事件"""
        success, message = self.nav_client.navigate_to_start()
        self.update_status(message)
    
    def on_go_end(self):
        """去到终点按钮点击事件"""
        success, message = self.nav_client.navigate_to_end()
        self.update_status(message)
    
    def update_status(self, status):
        """更新状态标签"""
        self.status_label.setText(f"状态：{status}")

    def add_log(self, text):
        # """添加日志信息"""
        # timestamp = rclpy.time.Time().now().nanoseconds / 1e9
        # self.log_area.append(f"[{timestamp:.2f}] {text}")
        # # 自动滚动到底部
        # # self.log_area.moveCursor(self.log_area.textCursor().End)

        # # 1. 获取当前时间
        # current_time = self.clock.now()
        # # 2. 转换为秒
        # timestamp = current_time.to_sec()
        # # 3. 追加日志
        # self.log_area.append(f"[{timestamp:.2f}] {text}")

         
        # 1. 获取当前本地时间
        current_time = datetime.datetime.now()
        # 2. 格式化时间（年-月-日 时:分:秒.毫秒）
        timestamp = current_time.strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]  # 保留3位毫秒
        # 3. 追加日志
        self.log_area.append(f"[{timestamp}] {text}")


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
    

# 主函数
def main(args=None):
    # 初始化ROS 2
    rclpy.init(args=args)
    
    # 创建导航客户端
    nav_client = Nav2Client()
    
    # 创建Qt应用
    app = QApplication(sys.argv)
    
    # 创建并显示GUI
    gui = Nav2GUI(nav_client)
    gui.show()
    
    # 运行Qt事件循环和ROS 2节点
    import threading
    def run_ros():
        rclpy.spin(nav_client)
    
    ros_thread = threading.Thread(target=run_ros, daemon=True)
    ros_thread.start()
    
    # 运行Qt应用
    try:
        app.exec()
    finally:
        # 清理资源
        nav_client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()