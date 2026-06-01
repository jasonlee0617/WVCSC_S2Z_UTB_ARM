import launch
import launch_ros.actions
import subprocess
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def is_node_running(node_name):
    """
    检查指定名称的节点是否正在运行
    :param node_name: 节点名称
    :return: 如果节点正在运行返回True，否则返回False
    """
    try:
        output = subprocess.check_output(['ros2', 'node', 'list']).decode('utf-8')
        return node_name in output
    except subprocess.CalledProcessError:
        return False


def generate_launch_description():

    ld = launch.LaunchDescription()

    node_name = 'can_bridge_node'
    # 检查节点是否已经在运行
    if not is_node_running(node_name):
        # 如果节点未运行，则启动该节点
        my_node = launch_ros.actions.Node(
            package='can_bridge',
            executable='can_bridge_node',
            name=node_name,
             output='screen'
        )
        ld.add_action(my_node)

    return ld
    