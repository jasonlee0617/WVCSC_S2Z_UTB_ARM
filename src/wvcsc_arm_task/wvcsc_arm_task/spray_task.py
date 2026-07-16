"""One-tree MoveIt observation, RGB alignment and spray coordinator."""

from dataclasses import dataclass
import math
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection2DArray
from wvcsc_interfaces.action import AlignTarget, ExecuteSpray, Spray

from .motion_state import MotionControlState
from .node_parameters import create_alicia_moveit
from .observation_pose import (
    camera_look_at_pose,
    tool_pose_from_camera_pose,
    transform_point,
)


@dataclass(frozen=True)
class FruitTarget:
    target_id: str
    confidence: float
    center_u: float
    center_v: float
    width: float
    height: float

    def iou(self, other):
        left = max(self.center_u - self.width / 2.0,
                   other.center_u - other.width / 2.0)
        top = max(self.center_v - self.height / 2.0,
                  other.center_v - other.height / 2.0)
        right = min(self.center_u + self.width / 2.0,
                    other.center_u + other.width / 2.0)
        bottom = min(self.center_v + self.height / 2.0,
                     other.center_v + other.height / 2.0)
        intersection = max(0.0, right - left) * max(0.0, bottom - top)
        union = self.width * self.height + other.width * other.height - intersection
        return 0.0 if union <= 0.0 else intersection / union

    def distance_to(self, other):
        return math.hypot(self.center_u - other.center_u,
                          self.center_v - other.center_v)


def detection_candidates(message, class_name, min_confidence):
    """Translate standard Detection2D messages into sorted task candidates."""
    candidates = []
    for detection in message.detections:
        if not detection.id or not detection.results:
            continue
        hypothesis = detection.results[0].hypothesis
        if (hypothesis.class_id != class_name or
                float(hypothesis.score) < float(min_confidence)):
            continue
        bbox = detection.bbox
        candidates.append(FruitTarget(
            detection.id,
            float(hypothesis.score),
            float(bbox.center.position.x),
            float(bbox.center.position.y),
            float(bbox.size_x),
            float(bbox.size_y),
        ))
    return sorted(candidates, key=lambda item: item.confidence, reverse=True)


class SprayTask(Node):
    _OBSERVATION_DISTANCES = (1.40, 1.30, 1.20, 1.10, 1.00, 0.90, 1.50)
    _OBSERVATION_POSITION_TOLERANCE = 0.02
    _OBSERVATION_ORIENTATION_TOLERANCE = 0.05

    def __init__(self):
        super().__init__('wvcsc_spray_task')
        self._declare_parameters()
        self._home = self._joint_parameter('home_pose')
        self._min_duration = float(self.get_parameter('min_spray_duration').value)
        self._max_duration = float(self.get_parameter('max_spray_duration').value)
        self._vision_timeout = float(self.get_parameter('vision_timeout_sec').value)
        self._downstream_server_timeout = float(
            self.get_parameter('downstream_server_timeout_sec').value)
        self._downstream_margin = float(
            self.get_parameter('downstream_result_margin_sec').value)
        self._tree_aim_height = float(self.get_parameter('tree_aim_height').value)
        self._camera_observation_height = float(
            self.get_parameter('camera_observation_height').value)
        self._observation_distance = float(
            self.get_parameter('observation_distance').value)
        self._camera_frame = str(self.get_parameter('camera_frame').value)
        if not 0.8 <= self._observation_distance <= 1.5:
            raise ValueError('observation_distance must be within 0.8 to 1.5 m')

        self.state = MotionControlState()
        self.arm, self._callback_group = create_alicia_moveit(self, self.state)
        self._base_frame = str(self.get_parameter('base_frame').value)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._vision_client = ActionClient(
            self, AlignTarget, str(self.get_parameter('vision_action_name').value),
            callback_group=self._callback_group)
        self._spray_client = ActionClient(
            self, Spray, str(self.get_parameter('spray_action_name').value),
            callback_group=self._callback_group)
        self._selected_target_pub = self.create_publisher(
            String, str(self.get_parameter('selected_target_topic').value), 10)
        self._motion_command_pub = self.create_publisher(
            String, '/motion_control/command', 10)

        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            Bool, str(self.get_parameter('motion_locked_topic').value),
            self._on_motion_locked, latched, callback_group=self._callback_group)
        self.create_subscription(
            Detection2DArray, str(self.get_parameter('tree_detection_topic').value),
            self._on_tree_detections, 10, callback_group=self._callback_group)
        self.create_subscription(
            Detection2DArray, str(self.get_parameter('fruit_detection_topic').value),
            self._on_fruit_detections, 10, callback_group=self._callback_group)

        self._action_server = ActionServer(
            self, ExecuteSpray, '/arm/execute_spray',
            execute_callback=self._execute_action,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._callback_group)
        self._abort = threading.Event()
        self._busy_mutex = threading.Lock()
        self._busy = False
        self._vision_mutex = threading.Lock()
        self._tree_frames = 0
        self._fruit_frames = 0
        self._fruit_counts = {}
        self._fruit_latest = {}
        self._observation_pose = None

    def _declare_parameters(self):
        parameters = {
            'home_pose': [0.0] * 6,
            'min_spray_duration': 0.2,
            'max_spray_duration': 10.0,
            'vision_action_name': '/vision/align_target',
            'vision_timeout_sec': 8.0,
            'spray_action_name': '/spray/execute',
            'downstream_server_timeout_sec': 2.0,
            'downstream_result_margin_sec': 2.0,
            'tree_detection_topic': '/vision/tree_detections',
            'fruit_detection_topic': '/vision/fruit_detections',
            'selected_target_topic': '/vision/selected_target_id',
            'motion_locked_topic': '/motion_control/locked',
            'tree_confidence': 0.50,
            'fruit_confidence': 0.50,
            'confirmation_frames': 3,
            'scan_timeout_sec': 3.0,
            'detection_timeout_sec': 2.0,
            'processed_iou_threshold': 0.30,
            'processed_center_distance_px': 40.0,
            'image_width': 1280,
            'image_height': 720,
            'base_frame': 'alicia_base_link',
            'tree_aim_height': 1.20,
            'camera_observation_height': 1.90,
            'observation_distance': 1.40,
            'camera_frame': 'camera_color_optical_frame',
        }
        for name, default in parameters.items():
            self.declare_parameter(name, default)

    def _joint_parameter(self, name):
        values = [float(value) for value in self.get_parameter(name).value]
        if len(values) != 6 or not all(math.isfinite(value) for value in values):
            raise ValueError(f'{name} must contain six finite joint positions')
        return values

    def _on_motion_locked(self, message):
        if message.data:
            self.state.stop()
            self._abort.set()
            self.arm.cancel()
        else:
            self.state.resume()
            if not self._is_busy():
                self._abort.clear()

    def _on_tree_detections(self, message):
        trees = detection_candidates(
            message, 'tree', self.get_parameter('tree_confidence').value)
        with self._vision_mutex:
            self._tree_frames = self._tree_frames + 1 if trees else 0

    def _on_fruit_detections(self, message):
        fruits = detection_candidates(
            message, 'diseased_fruit', self.get_parameter('fruit_confidence').value)
        with self._vision_mutex:
            self._fruit_frames += 1
            current = {fruit.target_id: fruit for fruit in fruits}
            self._fruit_counts = {
                target_id: self._fruit_counts.get(target_id, 0) + 1
                if target_id in current else 0
                for target_id in set(self._fruit_counts) | set(current)
            }
            self._fruit_latest = current

    def _goal_callback(self, request):
        error = self._validate_goal(request)
        if error or not self._claim():
            self.get_logger().warn(f'[ARM] rejected goal: {error or "busy or locked"}')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle):
        self._abort.set()
        self.arm.cancel()
        self._request_motion_stop()
        return CancelResponse.ACCEPT

    def _execute_action(self, goal_handle):
        request = goal_handle.request
        result = ExecuteSpray.Result()
        try:
            code, message = self._run_sequence(
                request,
                cancel_requested=lambda: goal_handle.is_cancel_requested,
                feedback=lambda phase, progress, text: self._feedback(
                    goal_handle, phase, progress, text))
            result.success = code in {
                ExecuteSpray.Result.OK,
                ExecuteSpray.Result.INSPECTED_NO_DISEASE,
            }
            result.error_code = code
            result.message = message
            if result.success:
                goal_handle.succeed()
            elif code == ExecuteSpray.Result.CANCELED and goal_handle.is_cancel_requested:
                goal_handle.canceled()
            else:
                goal_handle.abort()
            return result
        except Exception as error:
            self.get_logger().error(f'[ARM] internal error: {error}')
            result.error_code = ExecuteSpray.Result.INTERNAL_ERROR
            result.message = str(error)
            goal_handle.abort()
            return result
        finally:
            self._select_target('')
            self._release()

    def _run_sequence(self, request, cancel_requested, feedback):
        self._reset_vision()
        feedback(ExecuteSpray.Feedback.MOVING_TO_OBSERVE, 0.05, 'MOVING_TO_OBSERVE')
        if not self._move_to_observation(request.tree_hint):
            return self._observe_failure(cancel_requested)

        feedback(ExecuteSpray.Feedback.SCANNING_TREE, 0.15, 'SCANNING_TREE')
        if not self._wait_for_tree(cancel_requested):
            return self._vision_failure('tree was not confirmed in the camera view',
                                        cancel_requested)

        processed = []
        attempted = []
        sprayed = 0
        saw_disease = False
        while True:
            feedback(ExecuteSpray.Feedback.DETECTING_FRUITS, 0.25, 'DETECTING_FRUITS')
            candidates = self._wait_for_fruits(cancel_requested)
            if candidates is None:
                return self._vision_failure('fruit detector did not provide frames',
                                            cancel_requested)
            saw_disease = saw_disease or bool(candidates)
            feedback(ExecuteSpray.Feedback.QUEUING, 0.35, 'QUEUING')
            queue = self._queue(candidates, processed + attempted)
            if not queue:
                break
            target = queue[0]
            self._select_target(target.target_id)
            feedback(ExecuteSpray.Feedback.ALIGNING, 0.45, 'ALIGNING')
            ok, canceled, message = self._align_target(
                request.mission_id, request.tree_id, target.target_id, cancel_requested)
            if not ok:
                if canceled:
                    return ExecuteSpray.Result.CANCELED, message
                if message.startswith('[SAFETY]'):
                    self._request_motion_stop()
                    return ExecuteSpray.Result.INTERNAL_ERROR, message
                attempted.append(target)
                if not self._return_to_observation():
                    return ExecuteSpray.Result.HOME_FAILED, (
                        f'{message}; observation return failed')
                self._reset_fruit_tracking()
                continue

            feedback(ExecuteSpray.Feedback.SPRAYING, 0.60, 'SPRAYING')
            ok, canceled, message = self._spray_target(
                request.mission_id, request.tree_id, request.spray_duration,
                cancel_requested)
            if not ok:
                if canceled:
                    return ExecuteSpray.Result.CANCELED, message
                return self._spray_failure(message, cancel_requested)
            sprayed += 1
            processed.append(target)
            self._select_target('')
            feedback(ExecuteSpray.Feedback.RETURNING_TO_OBSERVE, 0.75,
                     'RETURNING_TO_OBSERVE')
            if not self._return_to_observation():
                return ExecuteSpray.Result.HOME_FAILED, 'observation return failed'
            self._reset_fruit_tracking()

        feedback(ExecuteSpray.Feedback.RETURNING_HOME, 0.90, 'RETURNING_HOME')
        if not self._return_home(cancel_requested):
            return (ExecuteSpray.Result.CANCELED, 'spray goal canceled') if self._aborted(
                cancel_requested) else (ExecuteSpray.Result.HOME_FAILED, 'HOME motion failed')
        feedback(ExecuteSpray.Feedback.COMPLETED, 1.0, 'COMPLETED')
        if sprayed:
            return ExecuteSpray.Result.OK, f'sprayed {sprayed} diseased fruit(s)'
        if saw_disease:
            return ExecuteSpray.Result.VISION_FAILED, 'diseased fruit could not be aligned'
        return ExecuteSpray.Result.INSPECTED_NO_DISEASE, 'tree inspected; no diseased fruit detected'

    def _observe_failure(self, cancel_requested):
        if self._aborted(cancel_requested):
            return ExecuteSpray.Result.CANCELED, 'spray goal canceled'
        if not self._return_home(cancel_requested):
            return ExecuteSpray.Result.HOME_FAILED, 'observation and HOME motion failed'
        return ExecuteSpray.Result.OBSERVE_FAILED, 'observation motion failed'

    def _vision_failure(self, message, cancel_requested):
        if self._aborted(cancel_requested):
            return ExecuteSpray.Result.CANCELED, 'spray goal canceled'
        if not self._return_home(cancel_requested):
            return ExecuteSpray.Result.HOME_FAILED, f'{message}; HOME motion failed'
        return ExecuteSpray.Result.VISION_FAILED, message

    def _spray_failure(self, message, cancel_requested):
        if self._aborted(cancel_requested):
            return ExecuteSpray.Result.CANCELED, 'spray goal canceled'
        if not self._return_home(cancel_requested):
            return ExecuteSpray.Result.HOME_FAILED, f'{message}; HOME motion failed'
        return ExecuteSpray.Result.SPRAY_FAILED, message

    def _wait_for_tree(self, cancel_requested):
        deadline = time.monotonic() + float(self.get_parameter('scan_timeout_sec').value)
        required = int(self.get_parameter('confirmation_frames').value)
        while time.monotonic() < deadline:
            if self._aborted(cancel_requested):
                return False
            with self._vision_mutex:
                if self._tree_frames >= required:
                    return True
            time.sleep(0.02)
        return False

    def _wait_for_fruits(self, cancel_requested):
        deadline = time.monotonic() + float(self.get_parameter('detection_timeout_sec').value)
        required = int(self.get_parameter('confirmation_frames').value)
        while time.monotonic() < deadline:
            if self._aborted(cancel_requested):
                return None
            with self._vision_mutex:
                if self._fruit_frames >= required:
                    return [
                        candidate for target_id, candidate in self._fruit_latest.items()
                        if self._fruit_counts.get(target_id, 0) >= required
                    ]
            time.sleep(0.02)
        with self._vision_mutex:
            return [] if self._fruit_frames else None

    def _queue(self, candidates, excluded):
        iou_threshold = float(self.get_parameter('processed_iou_threshold').value)
        distance_threshold = float(
            self.get_parameter('processed_center_distance_px').value)
        kept = [
            candidate for candidate in candidates
            if not any(
                candidate.iou(previous) >= iou_threshold or
                candidate.distance_to(previous) <= distance_threshold
                for previous in excluded)
        ]
        return sorted(
            kept,
            key=lambda item: (
                math.hypot(
                    item.center_u - float(self.get_parameter('image_width').value) / 2.0,
                    item.center_v - float(self.get_parameter('image_height').value) / 2.0),
                -item.confidence),
        )

    def _reset_vision(self):
        self._observation_pose = None
        self._reset_fruit_tracking()
        with self._vision_mutex:
            self._tree_frames = 0

    def _reset_fruit_tracking(self):
        with self._vision_mutex:
            self._fruit_frames = 0
            self._fruit_counts = {}
            self._fruit_latest = {}

    def _align_target(self, mission_id, tree_id, target_id, cancel_requested):
        goal = AlignTarget.Goal()
        goal.mission_id = mission_id
        goal.tree_id = tree_id
        goal.target_id = target_id
        goal.timeout = self._vision_timeout
        wrapped, canceled, error = self._run_downstream_action(
            self._vision_client, goal, self._vision_timeout + self._downstream_margin,
            cancel_requested, 'vision alignment')
        if wrapped is None:
            return False, canceled, error
        result = wrapped.result
        ok = wrapped.status == GoalStatus.STATUS_SUCCEEDED and result.success
        return ok, False, result.message or f'vision status={wrapped.status}'

    def _spray_target(self, mission_id, tree_id, duration, cancel_requested):
        goal = Spray.Goal()
        goal.mission_id = mission_id
        goal.tree_id = tree_id
        goal.duration = duration
        goal.mode = 'continuous'
        wrapped, canceled, error = self._run_downstream_action(
            self._spray_client, goal, duration + self._downstream_margin,
            cancel_requested, 'spray actuator')
        if wrapped is None:
            return False, canceled, error
        result = wrapped.result
        ok = wrapped.status == GoalStatus.STATUS_SUCCEEDED and result.success
        return ok, False, result.message or f'spray status={wrapped.status}'

    def _run_downstream_action(self, client, goal, result_timeout, cancel_requested, label):
        deadline = time.monotonic() + self._downstream_server_timeout
        while not client.server_is_ready():
            if self._aborted(cancel_requested):
                return None, True, f'{label} canceled'
            if time.monotonic() >= deadline:
                return None, False, f'{label} server is unavailable'
            time.sleep(0.02)
        response_future = client.send_goal_async(goal)
        response, canceled = self._wait_future(
            response_future, self._downstream_server_timeout, cancel_requested)
        if response is None:
            if canceled:
                response_future.add_done_callback(self._cancel_late_goal)
            return None, canceled, f'{label} goal response timed out or canceled'
        if not response.accepted:
            return None, False, f'{label} goal was rejected'
        result_future = response.get_result_async()
        wrapped, canceled = self._wait_future(
            result_future, result_timeout, cancel_requested, cancel_handle=response)
        if wrapped is None:
            return None, canceled, f'{label} result timed out or canceled'
        return wrapped, False, ''

    def _wait_future(self, future, timeout, cancel_requested, cancel_handle=None):
        deadline = time.monotonic() + timeout
        while not future.done():
            if self._aborted(cancel_requested) or time.monotonic() >= deadline:
                if cancel_handle is not None:
                    self._cancel_downstream_and_wait(cancel_handle, future)
                return None, self._aborted(cancel_requested)
            time.sleep(0.02)
        try:
            return future.result(), False
        except Exception:
            return None, False

    def _cancel_downstream_and_wait(self, goal_handle, result_future):
        deadline = time.monotonic() + self._downstream_server_timeout
        try:
            cancel_future = goal_handle.cancel_goal_async()
        except Exception:
            return False
        while not cancel_future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        while not result_future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        return result_future.done()

    @staticmethod
    def _cancel_late_goal(future):
        try:
            handle = future.result()
        except Exception:
            return
        if handle is not None and handle.accepted:
            handle.cancel_goal_async()

    @staticmethod
    def _hint_available(tree_hint):
        if tree_hint is None or not str(tree_hint.header.frame_id).strip():
            return False
        point = tree_hint.point
        return all(math.isfinite(value) for value in (point.x, point.y, point.z))

    def _move_to_observation(self, tree_hint):
        if not self._hint_available(tree_hint):
            self.get_logger().error('[ARM] tree_hint is required for observation')
            return False
        try:
            transform = self._tf_buffer.lookup_transform(
                self._base_frame, tree_hint.header.frame_id, rclpy.time.Time())
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            tree_in_base = transform_point(
                (tree_hint.point.x, tree_hint.point.y, tree_hint.point.z),
                (translation.x, translation.y, translation.z),
                (rotation.x, rotation.y, rotation.z, rotation.w))
            camera_transform = self._tf_buffer.lookup_transform(
                'tool0', self._camera_frame, rclpy.time.Time())
        except (TransformException, ValueError) as error:
            self.get_logger().error(f'[ARM] cannot build observation pose: {error}')
            return False
        camera_translation = camera_transform.transform.translation
        camera_rotation = camera_transform.transform.rotation
        distances = (self._observation_distance,) + tuple(
            distance for distance in self._OBSERVATION_DISTANCES
            if distance != self._observation_distance)
        for distance in distances:
            try:
                camera_position, camera_quat = camera_look_at_pose(
                    tree_in_base, self._tree_aim_height,
                    self._camera_observation_height, distance)
                tool_position, tool_quat = tool_pose_from_camera_pose(
                    camera_position, camera_quat,
                    (camera_translation.x, camera_translation.y, camera_translation.z),
                    (camera_rotation.x, camera_rotation.y,
                     camera_rotation.z, camera_rotation.w))
            except ValueError:
                continue
            if self._aborted(lambda: False):
                return False
            if self.arm.move_pose(
                    tool_position, tool_quat, frame_id=self._base_frame,
                    tolerance_position=self._OBSERVATION_POSITION_TOLERANCE,
                    tolerance_orientation=self._OBSERVATION_ORIENTATION_TOLERANCE):
                self._observation_pose = (tool_position, tool_quat)
                return True
        return False

    def _return_to_observation(self):
        if self._observation_pose is None or self._abort.is_set():
            return False
        position, quat = self._observation_pose
        return self.arm.move_pose(
            position, quat, frame_id=self._base_frame,
            tolerance_position=self._OBSERVATION_POSITION_TOLERANCE,
            tolerance_orientation=self._OBSERVATION_ORIENTATION_TOLERANCE)

    def _return_home(self, cancel_requested):
        return not self._aborted(cancel_requested) and self.arm.move_joints(self._home)

    def _select_target(self, target_id):
        message = String()
        message.data = target_id
        self._selected_target_pub.publish(message)

    def _request_motion_stop(self):
        message = String()
        message.data = 'stop'
        self._motion_command_pub.publish(message)

    def _aborted(self, cancel_requested):
        return self._abort.is_set() or cancel_requested()

    @staticmethod
    def _feedback(goal_handle, phase, progress, text):
        message = ExecuteSpray.Feedback()
        message.phase = phase
        message.progress = progress
        message.phase_text = text
        goal_handle.publish_feedback(message)

    def _validate_goal(self, request):
        if not str(request.mission_id).strip() or not str(request.tree_id).strip():
            return 'mission_id and tree_id are required'
        if request.spray_side not in ('left', 'right'):
            return 'spray_side must be left or right'
        if (not math.isfinite(float(request.spray_duration)) or
                not self._min_duration <= request.spray_duration <= self._max_duration):
            return 'spray_duration out of range'
        if not self._hint_available(request.tree_hint):
            return 'tree_hint in a named frame is required'
        return ''

    def _claim(self):
        with self._busy_mutex:
            if self._busy or self.state.locked:
                return False
            self._busy = True
            self._abort.clear()
            return True

    def _release(self):
        with self._busy_mutex:
            self._busy = False
            if not self.state.locked:
                self._abort.clear()

    def _is_busy(self):
        with self._busy_mutex:
            return self._busy


def main():
    rclpy.init()
    node = SprayTask()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
