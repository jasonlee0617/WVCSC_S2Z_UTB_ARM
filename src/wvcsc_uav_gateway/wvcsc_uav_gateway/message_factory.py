"""Conversion from validated mission dictionaries to ROS messages."""

from wvcsc_interfaces.msg import DiseaseTree, DiseaseTreeArray


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
