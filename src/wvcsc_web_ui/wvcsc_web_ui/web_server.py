"""FastAPI/WebSocket bridge limited to mission-level ROS interfaces."""

import asyncio
import os
import threading
import time

from ament_index_python.packages import get_package_share_directory
from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger
import uvicorn
from wvcsc_interfaces.msg import DiseaseTreeArray, MissionPlan, MissionStatus

from .state import MISSION_COMMANDS, SnapshotStore


class WebBridge(Node):
    def __init__(self, **kwargs):
        super().__init__('wvcsc_web_bridge', **kwargs)
        self.declare_parameter('host', '127.0.0.1')
        self.declare_parameter('port', 8080)
        self.declare_parameter('service_timeout_sec', 3.0)
        self.host = str(self.get_parameter('host').value)
        self.port = int(self.get_parameter('port').value)
        self._service_timeout = float(
            self.get_parameter('service_timeout_sec').value)
        self.store = SnapshotStore()
        group = ReentrantCallbackGroup()
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            MissionStatus, '/mission/status', self.store.update_status,
            latched, callback_group=group)
        self.create_subscription(
            DiseaseTreeArray, '/uav/disease_trees', self.store.update_mission,
            latched, callback_group=group)
        self.create_subscription(
            MissionPlan, '/mission/plan', self.store.update_plan,
            latched, callback_group=group)
        self._mission_clients = {
            command: self.create_client(
                Trigger, f'/mission/{command}', callback_group=group)
            for command in MISSION_COMMANDS
        }

    def call_command(self, command):
        if command not in self._mission_clients:
            return {'success': False, 'message': 'unsupported mission command'}
        client = self._mission_clients[command]
        if not client.wait_for_service(timeout_sec=self._service_timeout):
            return {
                'success': False,
                'message': f'/mission/{command} service is unavailable',
            }
        future = client.call_async(Trigger.Request())
        deadline = time.monotonic() + self._service_timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        if not future.done():
            return {'success': False, 'message': f'/mission/{command} timed out'}
        try:
            response = future.result()
        except Exception as error:
            return {'success': False, 'message': str(error)}
        return {'success': bool(response.success), 'message': response.message}


def execute_command(bridge, command):
    if command not in MISSION_COMMANDS:
        raise HTTPException(status_code=404, detail='unsupported mission command')
    result = bridge.call_command(command)
    if not result['success']:
        raise HTTPException(status_code=409, detail=result['message'])
    return result


def create_app(bridge, static_dir):
    app = FastAPI(title='WVCSC Mission UI', version='0.1.0')
    index_path = os.path.join(static_dir, 'index.html')

    @app.get('/')
    async def index():
        return FileResponse(index_path)

    @app.get('/api/snapshot')
    async def snapshot():
        return bridge.store.snapshot()

    @app.post('/api/mission/{command}')
    def mission_command(command: str):
        return execute_command(bridge, command)

    @app.websocket('/ws/status')
    async def status_socket(websocket: WebSocket):
        await websocket.accept()
        try:
            while True:
                await websocket.send_json(bridge.store.snapshot())
                await asyncio.sleep(0.25)
        except WebSocketDisconnect:
            return

    return app


def main():
    rclpy.init()
    node = WebBridge()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    static_dir = os.path.join(
        get_package_share_directory('wvcsc_web_ui'), 'static')
    app = create_app(node, static_dir)
    try:
        uvicorn.run(app, host=node.host, port=node.port, log_level='info')
    finally:
        executor.shutdown(timeout_sec=2.0)
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.try_shutdown()
