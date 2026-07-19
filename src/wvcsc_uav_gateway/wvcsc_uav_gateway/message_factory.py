"""Shared UAV message construction and publication helpers.

Mock and replay gateways intentionally publish the same latched mission topic.
Keeping its QoS contract here prevents the two input modes from drifting apart.
"""

from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from wvcsc_interfaces.msg import DiseaseTree, DiseaseTreeArray


def mission_publisher(node):
    """Create the common reliable/transient-local mission publisher."""
    qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    return node.create_publisher(DiseaseTreeArray, '/uav/disease_trees', qos)


def mission_message(config, stamp):
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
