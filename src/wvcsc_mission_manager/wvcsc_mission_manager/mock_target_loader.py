#!/usr/bin/env python3
"""Load simulation mock targets directly through ``/mission/load_manual``."""

import argparse
import math
from pathlib import Path
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from wvcsc_interfaces.msg import ManualMissionTarget
from wvcsc_interfaces.srv import LoadManualMission
import yaml


def _finite(value, label):
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f'{label} must be finite')
    return result


def _mapping(value, label):
    if not isinstance(value, dict):
        raise ValueError(f'{label} must be a mapping')
    return value


def load_mock_document(path):
    """Read and validate the simulation-only mock target YAML document."""
    source = Path(path).expanduser()
    with source.open(encoding='utf-8') as stream:
        document = yaml.safe_load(stream) or {}
    mission = _mapping(document.get('mission'), 'mission')
    if str(mission.get('mission_id', '')).strip() == '':
        raise ValueError('mission.mission_id is required')
    if str(mission.get('frame_id', '')).strip() != 'map':
        raise ValueError('mission.frame_id must be map')
    home = _mapping(mission.get('home_pose'), 'mission.home_pose')
    for key in ('x', 'y', 'yaw'):
        _finite(home.get(key), f'mission.home_pose.{key}')
    delay = _finite(mission.get('load_delay_sec'), 'mission.load_delay_sec')
    if delay < 0.0:
        raise ValueError('mission.load_delay_sec must be non-negative')
    trees = mission.get('trees')
    if not isinstance(trees, list) or not trees:
        raise ValueError('mission.trees must be a non-empty list')
    identifiers = set()
    for index, tree in enumerate(trees):
        tree = _mapping(tree, f'mission.trees[{index}]')
        tree_id = str(tree.get('tree_id', '')).strip()
        if not tree_id or tree_id in identifiers:
            raise ValueError('tree_id must be non-empty and unique')
        identifiers.add(tree_id)
        position = _mapping(tree.get('position'), f'{tree_id}.position')
        for key in ('x', 'y', 'z'):
            _finite(position.get(key), f'{tree_id}.position.{key}')
        confidence = _finite(tree.get('confidence'), f'{tree_id}.confidence')
        if not 0.0 < confidence <= 1.0:
            raise ValueError(f'{tree_id}.confidence must be in (0, 1]')
        if str(tree.get('spray_side', '')) not in ('left', 'right'):
            raise ValueError(f'{tree_id}.spray_side must be left or right')
        duration = _finite(tree.get('spray_duration'), f'{tree_id}.spray_duration')
        if duration <= 0.0:
            raise ValueError(f'{tree_id}.spray_duration must be positive')
    return document


def _set_pose(message, source):
    message.position.x = _finite(source['x'], 'pose.x')
    message.position.y = _finite(source['y'], 'pose.y')
    yaw = _finite(source['yaw'], 'pose.yaw')
    message.orientation.z = math.sin(yaw / 2.0)
    message.orientation.w = math.cos(yaw / 2.0)


def build_request(document, stamp):
    """Translate validated mock YAML into the common manual mission service."""
    mission = document['mission']
    request = LoadManualMission.Request()
    request.header.stamp = stamp
    request.header.frame_id = mission['frame_id']
    request.mission_id = str(mission['mission_id']).strip()
    request.return_home_after_finish = bool(
        mission.get('return_home_after_finish', False))
    _set_pose(request.home_pose, mission['home_pose'])
    for tree in mission['trees']:
        target = ManualMissionTarget()
        target.target_id = str(tree['tree_id']).strip()
        target.spray_side = str(tree['spray_side'])
        target.spray_duration = float(tree['spray_duration'])
        target.confidence = float(tree['confidence'])
        target.evidence_uri = str(tree.get('evidence_uri', '')).strip()
        target.tree_hint.x = float(tree['position']['x'])
        target.tree_hint.y = float(tree['position']['y'])
        target.tree_hint.z = float(tree['position']['z'])
        target.use_explicit_tree_hint = True
        target.compute_docking_pose = True
        request.targets.append(target)
    return request


class MockTargetLoader(Node):
    def __init__(self):
        super().__init__('wvcsc_mock_target_loader')
        self._client = self.create_client(LoadManualMission, '/mission/load_manual')

    def load(self, document, timeout_sec):
        if not self._client.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError('/mission/load_manual service is unavailable')
        request = build_request(document, self.get_clock().now().to_msg())
        future = self._client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        if not future.done():
            raise RuntimeError('/mission/load_manual timed out')
        response = future.result()
        if response is None or not response.success:
            message = response.message if response is not None else 'no response'
            raise RuntimeError(f'mission manager rejected mock mission: {message}')
        return response.message


def main():
    argv = remove_ros_args(args=sys.argv)[1:]
    if argv and argv[0] == '--':
        argv = argv[1:]
    parser = argparse.ArgumentParser()
    parser.add_argument('--file', required=True)
    parser.add_argument('--service-timeout-sec', type=float, default=30.0)
    args = parser.parse_args(argv)

    try:
        document = load_mock_document(args.file)
    except (OSError, ValueError, yaml.YAMLError) as error:
        print(f'[MOCK] invalid mock target file: {error}', file=sys.stderr)
        return 1

    rclpy.init(args=sys.argv)
    node = MockTargetLoader()
    try:
        delay = float(document['mission']['load_delay_sec'])
        if delay:
            node.get_logger().info(f'[MOCK] waiting {delay:.1f}s before loading mission')
            time.sleep(delay)
        message = node.load(document, args.service_timeout_sec)
        node.get_logger().info(
            f"[MOCK] loaded mission={document['mission']['mission_id']} "
            f"targets={len(document['mission']['trees'])}: {message}")
        return 0
    except RuntimeError as error:
        node.get_logger().error(f'[MOCK] mission load failed: {error}')
        return 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
