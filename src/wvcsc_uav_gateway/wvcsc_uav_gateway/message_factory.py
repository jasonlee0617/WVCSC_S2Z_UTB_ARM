# message_factory.py
# ============================================================================
# 无人机消息构造与发布的共享工厂模块
# ============================================================================
#
# 职责：
# 1. 统一创建带有 Transient Local QoS (持久化) 的任务发布器。
# 2. 将 YAML 字典数据安全地映射为 `DiseaseTreeArray` ROS 消息。
# 3. 确保 Mock 和 Replay 两种模式发布的 QoS 策略完全一致。
#

from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from wvcsc_interfaces.msg import DiseaseTree, DiseaseTreeArray


def mission_publisher(node):
    """
    创建统一的无人机任务发布器。

    【关键工程细节】：
    使用 `DurabilityPolicy.TRANSIENT_LOCAL` (持久化)。
    这保证了即使在 Gazebo 仿真中，任务管理器（或 Web UI）比无人机节点晚启动，
    也能立刻接收到上一个发布的任务，而不会因为启动时序问题“错过”任务。
    """
    qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    return node.create_publisher(DiseaseTreeArray, '/uav/disease_trees', qos)


def mission_message(config, stamp):
    """
    将配置字典转换为标准的 `DiseaseTreeArray` ROS 消息。
    """
    message = DiseaseTreeArray()
    message.header.stamp = stamp
    message.header.frame_id = config['frame_id']
    message.mission_id = config['mission_id']
    message.source_mode = config['source_mode']

    for item in config['trees']:
        tree = DiseaseTree()
        tree.tree_id = item['tree_id']
        tree.confidence = item['confidence']
        tree.position.x = item['position']['x']
        tree.position.y = item['position']['y']
        tree.position.z = item['position']['z']
        tree.spray_side = item['spray_side']
        tree.spray_duration = item['spray_duration']
        tree.evidence_uri = item['evidence_uri']
        message.trees.append(tree)
    return message