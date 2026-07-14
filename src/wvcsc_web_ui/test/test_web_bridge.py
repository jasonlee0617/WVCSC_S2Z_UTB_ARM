import os

os.environ['ROS_DOMAIN_ID'] = '83'
os.environ.setdefault('ROS_LOG_DIR', '/tmp/wvcsc_web_ui_test_logs')

import rclpy
from rclpy.context import Context

from wvcsc_web_ui.state import MISSION_COMMANDS
from wvcsc_web_ui.web_server import WebBridge


def test_bridge_keeps_ros_clients_in_node_registry():
    context = Context()
    rclpy.init(context=context)
    node = WebBridge(context=context)
    try:
        assert set(node._mission_clients) == set(MISSION_COMMANDS)
        clients = list(node.clients)
        assert len(clients) == len(MISSION_COMMANDS)
        assert all(hasattr(client, 'service_is_ready') for client in clients)
    finally:
        node.destroy_node()
        context.try_shutdown()
