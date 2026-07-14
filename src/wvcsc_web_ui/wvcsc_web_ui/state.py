"""Thread-safe JSON snapshots for the Web bridge."""

from copy import deepcopy
import threading
import time


MISSION_COMMANDS = (
    'start', 'pause', 'resume', 'skip_current', 'return_home', 'cancel',
    'reset')


def _stamp(message):
    stamp = getattr(getattr(message, 'header', None), 'stamp', None)
    if stamp is None:
        return {'sec': 0, 'nanosec': 0}
    return {
        'sec': int(getattr(stamp, 'sec', 0)),
        'nanosec': int(getattr(stamp, 'nanosec', 0)),
    }


def mission_status_to_dict(message):
    return {
        'stamp': _stamp(message),
        'mission_id': message.mission_id,
        'state': int(message.state),
        'state_text': message.state_text,
        'current_tree_id': message.current_tree_id,
        'current_index': int(message.current_index),
        'total_targets': int(message.total_targets),
        'completed_targets': int(message.completed_targets),
        'skipped_targets': int(message.skipped_targets),
        'target_statuses': [{
            'tree_id': item.tree_id,
            'state': int(item.state),
            'state_text': item.state_text,
            'message': item.message,
        } for item in message.target_statuses],
        'last_error': message.last_error,
        'nav_goal_active': bool(message.nav_goal_active),
        'arm_goal_active': bool(message.arm_goal_active),
    }


def disease_tree_array_to_dict(message):
    trees = []
    for tree in message.trees:
        trees.append({
            'tree_id': tree.tree_id,
            'confidence': float(tree.confidence),
            'position': {
                'x': float(tree.position.x),
                'y': float(tree.position.y),
                'z': float(tree.position.z),
            },
            'spray_side': tree.spray_side,
            'spray_duration': float(tree.spray_duration),
            'evidence_uri': tree.evidence_uri,
        })
    return {
        'stamp': _stamp(message),
        'frame_id': message.header.frame_id,
        'mission_id': message.mission_id,
        'source_mode': message.source_mode,
        'trees': trees,
    }


def _pose(pose):
    return {
        'position': {
            'x': float(pose.position.x),
            'y': float(pose.position.y),
            'z': float(pose.position.z),
        },
        'orientation': {
            'x': float(pose.orientation.x),
            'y': float(pose.orientation.y),
            'z': float(pose.orientation.z),
            'w': float(pose.orientation.w),
        },
    }


def mission_plan_to_dict(message):
    return {
        'stamp': _stamp(message),
        'frame_id': message.header.frame_id,
        'mission_id': message.mission_id,
        'return_home_after_finish': bool(message.return_home_after_finish),
        'home_pose': _pose(message.home_pose),
        'targets': [{
            'tree_id': item.target.tree_id,
            'tree_position': {
                'x': float(item.target.position.x),
                'y': float(item.target.position.y),
                'z': float(item.target.position.z),
            },
            'docking_pose': _pose(item.docking_pose),
        } for item in message.targets],
    }


class SnapshotStore:
    def __init__(self):
        self._lock = threading.Lock()
        self._generation = 0
        self._status = None
        self._status_updated_at = None
        self._mission = None
        self._plan = None

    def update_status(self, message):
        with self._lock:
            self._status = mission_status_to_dict(message)
            self._status_updated_at = time.monotonic()
            self._generation += 1

    def update_mission(self, message):
        with self._lock:
            self._mission = disease_tree_array_to_dict(message)
            self._generation += 1

    def update_plan(self, message):
        with self._lock:
            self._plan = mission_plan_to_dict(message)
            self._generation += 1

    def snapshot(self):
        with self._lock:
            status_age = (
                None if self._status_updated_at is None
                else max(0.0, time.monotonic() - self._status_updated_at)
            )
            return deepcopy({
                'generation': self._generation,
                'connected': status_age is not None and status_age <= 2.0,
                'status_age_sec': status_age,
                'status': self._status,
                'mission': self._mission,
                'plan': self._plan,
            })
