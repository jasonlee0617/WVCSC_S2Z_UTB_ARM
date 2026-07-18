"""One-tree MoveIt observation, RGB alignment and spray coordinator."""

from dataclasses import dataclass, field
import json
import math
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data)
from sensor_msgs.msg import CameraInfo, JointState
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection2DArray
from wvcsc_interfaces.action import AlignTarget, ExecuteSpray, Spray
from wvcsc_interfaces.msg import Target2D

from .motion_state import MotionControlState
from .node_parameters import create_alicia_moveit
from .observation_optimizer import ObservationCandidate, ObservationOptimizer
from .observation_pose import (
    recenter_camera_pose, tool_pose_from_camera_pose, transform_point)


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


@dataclass
class TargetAttempt:
    target: FruitTarget
    count: int = 0
    recentered_observation_indices: set = field(default_factory=set)


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


def deduplicate_candidates(
        candidates, iou_threshold=0.35, center_distance_px=10.0):
    """Keep only the strongest task candidate for one physical fruit."""
    unique = []
    for candidate in sorted(
            candidates, key=lambda item: item.confidence, reverse=True):
        if any(
                candidate.iou(previous) >= iou_threshold or
                candidate.distance_to(previous) <= center_distance_px
                for previous in unique):
            continue
        unique.append(candidate)
    return unique


def spray_summary(
        detected, sprayed, unresolved, alignment_failures, recenter_attempts,
        recenter_failures, alignment_attempts):
    return (
        f'detected={detected} sprayed={sprayed} unresolved={unresolved} '
        f'alignment_failures={alignment_failures} '
        f'recenter_attempts={recenter_attempts} '
        f'recenter_failures={recenter_failures} '
        f'alignment_attempts={alignment_attempts}')


def target_accounting_is_complete(detected, sprayed, unresolved):
    return int(detected) == int(sprayed) + int(unresolved)


def final_spray_outcome(sprayed, unresolved, saw_disease, summary):
    if sprayed and unresolved:
        return ExecuteSpray.Result.PARTIAL_SUCCESS, summary
    if sprayed:
        return ExecuteSpray.Result.OK, summary
    if saw_disease:
        return ExecuteSpray.Result.VISION_FAILED, summary
    return (
        ExecuteSpray.Result.INSPECTED_NO_DISEASE,
        f'{summary}; tree inspected; no diseased fruit detected')


def completion_feedback_allowed(result_code):
    return result_code in {
        ExecuteSpray.Result.OK,
        ExecuteSpray.Result.PARTIAL_SUCCESS,
        ExecuteSpray.Result.INSPECTED_NO_DISEASE,
    }


def target_pixel_error(
        center_u, center_v, image_width, image_height, desired_offset_u_px,
        desired_offset_v_px):
    desired_u = float(image_width) / 2.0 + float(desired_offset_u_px)
    desired_v = float(image_height) / 2.0 + float(desired_offset_v_px)
    return float(center_u) - desired_u, float(center_v) - desired_v


def target_requires_recenter(
        center_u, center_v, image_width, image_height, desired_offset_u_px,
        desired_offset_v_px, maximum_error_px):
    """Return whether either image axis is outside the servo fine-workspace."""
    error_u, error_v = target_pixel_error(
        center_u, center_v, image_width, image_height, desired_offset_u_px,
        desired_offset_v_px)
    return (abs(error_u) > float(maximum_error_px) or
            abs(error_v) > float(maximum_error_px))


class SprayTask(Node):
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
        self._camera_frame = str(self.get_parameter('camera_frame').value)
        self._base_frame = str(self.get_parameter('base_frame').value)
        self._observation_config = self._observation_parameters()
        self._recenter_config = self._target_recenter_parameters()
        self._observation_optimizer = ObservationOptimizer(
            self.get_parameter('robot_description').value,
            self._base_frame,
            'tool0',
            self.arm_joint_names,
            self._observation_config)
        if int(self.get_parameter('max_alignment_attempts').value) <= 0:
            raise ValueError('max_alignment_attempts must be positive')

        self.state = MotionControlState()
        self.arm, self._callback_group = create_alicia_moveit(self, self.state)
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
        self._observation_debug_pub = self.create_publisher(
            String, str(self.get_parameter('observation_debug_topic').value), 10)

        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._inference_mode_pub = self.create_publisher(
            String, str(self.get_parameter('inference_mode_topic').value), latched)
        self.create_subscription(
            Bool, str(self.get_parameter('motion_locked_topic').value),
            self._on_motion_locked, latched, callback_group=self._callback_group)
        self.create_subscription(
            Detection2DArray, str(self.get_parameter('tree_detection_topic').value),
            self._on_tree_detections, 10, callback_group=self._callback_group)
        self.create_subscription(
            Detection2DArray, str(self.get_parameter('fruit_detection_topic').value),
            self._on_fruit_detections, 10, callback_group=self._callback_group)
        self.create_subscription(
            Target2D, str(self.get_parameter('vision_target_topic').value),
            self._on_selected_target, 10, callback_group=self._callback_group)
        self.create_subscription(
            CameraInfo, str(self.get_parameter('camera_info_topic').value),
            self._on_camera_info, qos_profile_sensor_data,
            callback_group=self._callback_group)
        self.create_subscription(
            JointState, str(self.get_parameter('joint_state_topic').value),
            self._on_joint_state, qos_profile_sensor_data,
            callback_group=self._callback_group)

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
        self._state_mutex = threading.Lock()
        self._tree_frames = 0
        self._fruit_frames = 0
        self._fruit_counts = {}
        self._fruit_latest = {}
        self._target_confirmation_id = ''
        self._target_valid_frames = 0
        self._target_confirmation_frames = 0
        self._target_workspace_stable_since = None
        self._target_workspace_last_seen = None
        self._latest_selected_target = None
        self._observation_pose = None
        self._observation_candidates = []
        self._observation_candidate_index = -1
        self._observation_distance = None
        self._tree_in_base = None
        self._camera_mount = None
        self._camera_model = None
        self._joint_positions = None
        self._joint_state_sequence = 0
        self._active_mission = ''
        self._active_tree = ''

    @property
    def arm_joint_names(self):
        return ('joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6')

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
            'vision_target_topic': '/vision/target',
            'selected_target_topic': '/vision/selected_target_id',
            'inference_mode_topic': '/vision/inference_mode',
            'motion_locked_topic': '/motion_control/locked',
            'tree_confidence': 0.10,
            'fruit_confidence': 0.20,
            'confirmation_frames': 3,
            # Real tree YOLO needs several frames to satisfy confirmation_frames.
            # One second can expire before its first complete inference.
            'scan_pose_detection_timeout_sec': 5.0,
            'detection_timeout_sec': 2.0,
            'fruit_collection_settle_sec': 1.00,
            'max_alignment_attempts': 2,
            'target_recenter_trigger_px': 48.0,
            'target_recenter_max_angle_deg': 20.0,
            'target_recenter_refine_goal_px': 8.0,
            'target_recenter_max_iterations': 2,
            'target_recenter_residual_candidates_px': [
                0.0, 8.0, 12.0, 16.0, 24.0, 32.0, 40.0],
            # Must match wvcsc_visual_servo/config/visual_servo.yaml.
            'target_recenter_desired_offset_u_px': 0.0,
            'target_recenter_desired_offset_v_px': 28.0,
            'target_post_recenter_stable_sec': 0.50,
            'target_post_recenter_min_confidence': 0.30,
            'processed_iou_threshold': 0.30,
            'processed_center_distance_px': 40.0,
            'image_width': 1280,
            'image_height': 720,
            'base_frame': 'alicia_base_link',
            'camera_frame': 'camera_color_optical_frame',
            'camera_info_topic': '/camera/camera/color/camera_info',
            'joint_state_topic': '/joint_states',
            'robot_description': '',
            'observation_debug_topic': '/arm/observation_debug',
            'observation_input_timeout_sec': 2.0,
            'observation_search_timeout_sec': 8.0,
            'observation_max_plans': 8,
            'fruit_zone_height_min_m': 0.70,
            'fruit_zone_height_max_m': 1.70,
            'fruit_zone_radius_m': 0.50,
            'observation_distance_min_m': 0.90,
            'observation_distance_max_m': 1.50,
            'observation_distance_step_m': 0.10,
            'camera_height_min_m': 1.45,
            'camera_height_max_m': 1.75,
            'camera_height_step_m': 0.10,
            'observation_azimuth_offsets_deg': [0.0, -12.0, 12.0],
            'observation_image_margin_ratio': 0.07,
            # Keep a margin below MoveIt Servo's 17.0 singularity slowdown
            # threshold while retaining a second physically distinct recovery view.
            'observation_max_condition_number': 16.5,
            'observation_min_joint_margin_rad': 0.22,
            'observation_preferred_joint_margin_rad': 0.35,
            'observation_position_tolerance_m': 0.01,
            'observation_orientation_tolerance_rad': 0.01,
        }
        for name, default in parameters.items():
            self.declare_parameter(name, default)

    def _joint_parameter(self, name):
        values = [float(value) for value in self.get_parameter(name).value]
        if len(values) != 6 or not all(math.isfinite(value) for value in values):
            raise ValueError(f'{name} must contain six finite joint positions')
        return values

    def _observation_parameters(self):
        values = {
            'fruit_zone_height_min_m': float(
                self.get_parameter('fruit_zone_height_min_m').value),
            'fruit_zone_height_max_m': float(
                self.get_parameter('fruit_zone_height_max_m').value),
            'fruit_zone_radius_m': float(
                self.get_parameter('fruit_zone_radius_m').value),
            'distance_min_m': float(
                self.get_parameter('observation_distance_min_m').value),
            'distance_max_m': float(
                self.get_parameter('observation_distance_max_m').value),
            'distance_step_m': float(
                self.get_parameter('observation_distance_step_m').value),
            'camera_height_min_m': float(
                self.get_parameter('camera_height_min_m').value),
            'camera_height_max_m': float(
                self.get_parameter('camera_height_max_m').value),
            'camera_height_step_m': float(
                self.get_parameter('camera_height_step_m').value),
            'azimuth_offsets_deg': tuple(
                float(value) for value in
                self.get_parameter('observation_azimuth_offsets_deg').value),
            'image_margin_ratio': float(
                self.get_parameter('observation_image_margin_ratio').value),
            'max_condition_number': float(
                self.get_parameter('observation_max_condition_number').value),
            'min_joint_margin_rad': float(
                self.get_parameter('observation_min_joint_margin_rad').value),
            'preferred_joint_margin_rad': float(
                self.get_parameter(
                    'observation_preferred_joint_margin_rad').value),
            'position_tolerance_m': float(
                self.get_parameter(
                    'observation_position_tolerance_m').value),
            'orientation_tolerance_rad': float(
                self.get_parameter(
                    'observation_orientation_tolerance_rad').value),
        }
        positive = (
            'fruit_zone_height_min_m', 'fruit_zone_height_max_m',
            'fruit_zone_radius_m', 'distance_min_m', 'distance_max_m',
            'distance_step_m', 'camera_height_min_m', 'camera_height_max_m',
            'camera_height_step_m', 'max_condition_number',
            'min_joint_margin_rad', 'preferred_joint_margin_rad',
            'position_tolerance_m', 'orientation_tolerance_rad')
        if (not all(math.isfinite(values[name]) and values[name] > 0.0
                    for name in positive) or
                values['fruit_zone_height_min_m'] >=
                values['fruit_zone_height_max_m'] or
                values['distance_min_m'] > values['distance_max_m'] or
                values['camera_height_min_m'] > values['camera_height_max_m'] or
                values['preferred_joint_margin_rad'] <
                values['min_joint_margin_rad'] or
                not values['azimuth_offsets_deg'] or
                not 0.0 <= values['image_margin_ratio'] < 0.5):
            raise ValueError('observation search parameters are invalid')
        return values

    def _target_recenter_parameters(self):
        values = {
            'trigger_px': float(self.get_parameter('target_recenter_trigger_px').value),
            'max_angle_deg': float(self.get_parameter('target_recenter_max_angle_deg').value),
            'refine_goal_px': float(
                self.get_parameter('target_recenter_refine_goal_px').value),
            'max_iterations': int(
                self.get_parameter('target_recenter_max_iterations').value),
            'residual_candidates_px': tuple(sorted(set(
                float(value) for value in self.get_parameter(
                    'target_recenter_residual_candidates_px').value))),
            'desired_offset_u_px': float(
                self.get_parameter('target_recenter_desired_offset_u_px').value),
            'desired_offset_v_px': float(
                self.get_parameter('target_recenter_desired_offset_v_px').value),
            'post_stable_sec': float(
                self.get_parameter('target_post_recenter_stable_sec').value),
            'post_min_confidence': float(
                self.get_parameter(
                    'target_post_recenter_min_confidence').value),
        }
        scalar_values = tuple(
            value for name, value in values.items()
            if name != 'residual_candidates_px')
        if (not all(math.isfinite(value) for value in scalar_values) or
                values['trigger_px'] <= 0.0 or values['max_angle_deg'] <= 0.0 or
                values['max_angle_deg'] > 180.0 or
                values['refine_goal_px'] <= 0.0 or
                values['refine_goal_px'] >= values['trigger_px'] or
                not 1 <= values['max_iterations'] <= 3 or
                not values['residual_candidates_px'] or
                any(not math.isfinite(value) or value < 0.0 or
                    value >= values['trigger_px']
                    for value in values['residual_candidates_px']) or
                values['post_stable_sec'] <= 0.0 or
                not 0.0 <= values['post_min_confidence'] <= 1.0):
            raise ValueError('target recenter parameters are invalid')
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

    def _on_selected_target(self, message):
        with self._vision_mutex:
            if not (
                    message.valid and
                    message.target_id == self._target_confirmation_id and
                    message.image_width > 0 and message.image_height > 0):
                self._target_valid_frames = 0
                self._target_confirmation_frames = 0
                self._target_workspace_stable_since = None
                self._target_workspace_last_seen = None
                return
            self._latest_selected_target = FruitTarget(
                message.target_id,
                float(message.confidence),
                float(message.center_u),
                float(message.center_v),
                float(message.width),
                float(message.height),
            )
            self._target_valid_frames += 1
            reliable_in_workspace = (
                math.isfinite(message.confidence)
                and float(message.confidence) >=
                self._recenter_config['post_min_confidence']
                and not target_requires_recenter(
                    message.center_u, message.center_v, message.image_width,
                    message.image_height,
                    self._recenter_config['desired_offset_u_px'],
                    self._recenter_config['desired_offset_v_px'],
                    self._recenter_config['trigger_px']))
            if reliable_in_workspace:
                now = time.monotonic()
                if self._target_workspace_stable_since is None:
                    self._target_workspace_stable_since = now
                self._target_workspace_last_seen = now
                self._target_confirmation_frames += 1
            else:
                self._target_confirmation_frames = 0
                self._target_workspace_stable_since = None
                self._target_workspace_last_seen = None

    def _on_camera_info(self, message):
        if message.width <= 0 or message.height <= 0:
            return
        fx, fy, cx, cy = (
            float(message.k[0]), float(message.k[4]),
            float(message.k[2]), float(message.k[5]))
        if min(fx, fy) <= 0.0:
            return
        with self._state_mutex:
            self._camera_model = (fx, fy, cx, cy, int(message.width), int(message.height))

    def _on_joint_state(self, message):
        values = dict(zip(message.name, message.position))
        try:
            joints = tuple(float(values[name]) for name in self.arm_joint_names)
        except KeyError:
            return
        if not all(math.isfinite(value) for value in joints):
            return
        with self._state_mutex:
            self._joint_positions = joints
            self._joint_state_sequence += 1

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
                ExecuteSpray.Result.PARTIAL_SUCCESS,
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
            self._set_inference_mode('idle')
            self._active_mission = ''
            self._active_tree = ''
            self._release()

    def _run_sequence(self, request, cancel_requested, feedback):
        tree = str(request.tree_id).strip()
        self._active_mission = str(request.mission_id).strip()
        self._active_tree = tree
        self.get_logger().info(
            f'[ARM][{tree}] GOAL_ACCEPTED mission={request.mission_id.strip()} '
            f'side={request.spray_side} spray_duration={request.spray_duration:.1f}s')
        self._reset_vision()
        self._set_inference_mode('idle')
        self.get_logger().info(f'[ARM][{tree}] OBSERVE computing look-at pose from tree_hint...')
        feedback(ExecuteSpray.Feedback.MOVING_TO_OBSERVE, 0.05, 'MOVING_TO_OBSERVE')
        if not self._move_to_observation(request.tree_hint):
            return self._observe_failure(cancel_requested)
        self.get_logger().info(
            f'[ARM][{tree}] OBSERVE selected distance={self._observation_distance}m '
            f'index={self._observation_candidate_index} '
            f'tree_in_base=({self._tree_in_base[0]:.2f},{self._tree_in_base[1]:.2f},'
            f'{self._tree_in_base[2]:.2f})')

        feedback(ExecuteSpray.Feedback.SCANNING_TREE, 0.15, 'SCANNING_TREE')
        self.get_logger().info(
            f'[ARM][{tree}] SCAN inference_mode=tree '
            f'conf={float(self.get_parameter("tree_confidence").value):.2f}')
        if not self._scan_for_tree(cancel_requested):
            return self._vision_failure('tree was not confirmed in the camera view',
                                        cancel_requested)
        self.get_logger().info(f'[ARM][{tree}] SCAN tree confirmed')

        processed = []
        exhausted = []
        known_targets = []
        attempts = []
        pending_attempt = None
        sprayed = 0
        saw_disease = False
        alignment_failures = 0
        recenter_attempts = 0
        recenter_failures = 0
        alignment_attempts = 0
        while True:
            self._set_inference_mode('fruits')
            feedback(ExecuteSpray.Feedback.DETECTING_FRUITS, 0.25, 'DETECTING_FRUITS')
            self.get_logger().info(
                f'[ARM][{tree}] DETECT inference_mode=fruits '
                f'timeout={float(self.get_parameter("detection_timeout_sec").value):.1f}s '
                f'confirmation={int(self.get_parameter("confirmation_frames").value)}')
            candidates = self._wait_for_fruits(cancel_requested)
            if candidates is None:
                return self._vision_failure('fruit detector did not provide frames',
                                            cancel_requested)
            self.get_logger().info(
                f'[ARM][{tree}] DETECT found={len(candidates)} stable candidates '
                f'ids=({",".join(c.target_id for c in candidates[:8])})'
                f'{"..." if len(candidates) > 8 else ""}')
            saw_disease = saw_disease or bool(candidates)
            feedback(ExecuteSpray.Feedback.QUEUING, 0.35, 'QUEUING')
            queue = self._queue(candidates, processed + exhausted)
            if pending_attempt is not None and queue:
                target = min(
                    queue, key=lambda item: item.distance_to(pending_attempt.target))
                self._replace_known_target(
                    known_targets, pending_attempt.target, target)
                self._remember_targets(
                    known_targets,
                    [candidate for candidate in candidates if candidate is not target])
            else:
                self._remember_targets(known_targets, candidates)
            self.get_logger().info(
                f'[ARM][{tree}] QUEUE candidates={len(candidates)} '
                f'processed={len(processed)} exhausted={len(exhausted)} '
                f'→ queued={len(queue)}')
            if not queue:
                if pending_attempt is not None:
                    self._mark_unresolved(pending_attempt.target, exhausted)
                    self.get_logger().warn(
                        f'[ARM][ALIGN] target={pending_attempt.target.target_id} '
                        'was not redetected after observation recovery; marked unresolved')
                    pending_attempt = None
                for target in self._pending_targets(
                        known_targets, processed, exhausted):
                    self._mark_unresolved(target, exhausted)
                    self.get_logger().warn(
                        f'[ARM][QUEUE] target={target.target_id} disappeared '
                        'after a complete detection cycle; marked unresolved')
                self.get_logger().info(
                    f'[ARM][{tree}] DETECT queue empty '
                    f'(processed={len(processed)} exhausted={len(exhausted)}) → breaking loop')
                break
            if pending_attempt is not None:
                attempt = pending_attempt
                attempt.target = target
                pending_attempt = None
            else:
                target = queue[0]
                attempt = self._attempt_for(target, attempts)
                if attempt is None:
                    attempt = TargetAttempt(target)
                    attempts.append(attempt)
            feedback(ExecuteSpray.Feedback.ALIGNING, 0.40, 'LOCKING_TARGET')
            locked_target = self._lock_target(target.target_id, cancel_requested)
            recenter_attempts += 1
            if locked_target is None:
                recentered = False
                recenter_message = 'target was not locked before recenter'
            else:
                feedback(ExecuteSpray.Feedback.ALIGNING, 0.42, 'RECENTERING_TARGET')
                recentered, recenter_message = self._recenter_target(
                    locked_target, attempt, cancel_requested)
            if not recentered:
                recenter_failures += 1
                if self._aborted(cancel_requested):
                    return ExecuteSpray.Result.CANCELED, 'spray goal canceled'
                self._select_target('')
                self._set_inference_mode('idle')
                self.get_logger().warn(
                    f'[ARM][RECENTER] target={target.target_id} {recenter_message}; '
                    'trying the next observation candidate')
                self._rewind_for_untried_observation(attempt)
                recovered, moved = self._recover_to_next_observation(
                    cancel_requested, feedback)
                if recovered:
                    pending_attempt = attempt
                    self._reset_fruit_tracking()
                    continue
                if moved:
                    return self._alignment_recovery_failure(
                        f'target recenter failed: {recenter_message}; '
                        'tree reconfirmation failed after observation recovery',
                        cancel_requested)
                self._mark_unresolved(attempt.target, exhausted)
                feedback(ExecuteSpray.Feedback.RETURNING_TO_OBSERVE, 0.40,
                         'RETURNING_TO_OBSERVE')
                if not self._return_to_observation():
                    return self._alignment_recovery_failure(
                        f'target recenter failed: {recenter_message}; '
                        'observation recovery failed', cancel_requested)
                self._reset_fruit_tracking()
                continue
            attempt.count += 1
            alignment_attempts += 1
            feedback(ExecuteSpray.Feedback.ALIGNING, 0.45, 'ALIGNING')
            self.get_logger().info(
                f'[ARM][ALIGN] start target={target.target_id} '
                f'attempt={attempt.count}/'
                f'{int(self.get_parameter("max_alignment_attempts").value)} '
                f'observation_distance={self._observation_distance}')
            ok, canceled, align_code, message = self._align_target(
                request.mission_id, request.tree_id, target.target_id, cancel_requested)
            self.get_logger().info(
                f'[ARM][ALIGN] result target={target.target_id} '
                f'code={align_code} message={message}')
            if not ok:
                alignment_failures += 1
                if canceled:
                    return ExecuteSpray.Result.CANCELED, message
                if align_code == AlignTarget.Result.SERVO_SAFETY_STOP:
                    safety_message = (
                        f'[SAFETY] visual alignment code={align_code}: {message}')
                    self.get_logger().error(f'[ARM][ALIGN] {safety_message}')
                    self._request_motion_stop()
                    return ExecuteSpray.Result.INTERNAL_ERROR, safety_message
                recoverable = align_code in {
                    AlignTarget.Result.TIMEOUT,
                    AlignTarget.Result.TARGET_STALE,
                    AlignTarget.Result.SERVO_SINGULARITY,
                }
                if not recoverable:
                    return self._alignment_recovery_failure(
                        f'visual alignment code={align_code}: {message}',
                        cancel_requested)
                self._select_target('')
                self._set_inference_mode('idle')
                if self._alignment_retry_allowed(attempt.count):
                    self.get_logger().warn(
                        f'[ARM][ALIGN] recoverable failure code={align_code}; '
                        'trying the next observation candidate')
                    self._rewind_for_untried_observation(attempt)
                    recovered, moved = self._recover_to_next_observation(
                        cancel_requested, feedback)
                    if recovered:
                        pending_attempt = attempt
                        self._reset_fruit_tracking()
                        self.get_logger().info(
                            f'[ARM][ALIGN] recovery ready at '
                            f'{self._observation_distance} m; redetecting fruit')
                        continue
                    if moved:
                        return self._alignment_recovery_failure(
                            f'visual alignment code={align_code}: {message}; '
                            'tree reconfirmation failed after observation recovery',
                            cancel_requested)
                self._mark_unresolved(attempt.target, exhausted)
                feedback(ExecuteSpray.Feedback.RETURNING_TO_OBSERVE, 0.40,
                         'RETURNING_TO_OBSERVE')
                if not self._return_to_observation():
                    return self._alignment_recovery_failure(
                        f'visual alignment code={align_code}: {message}; '
                        'observation recovery failed', cancel_requested)
                self.get_logger().warn(
                    f'[ARM][ALIGN] exhausted target={target.target_id} '
                    f'after {attempt.count} attempt(s)')
                self._reset_fruit_tracking()
                continue

            self._set_inference_mode('idle')
            feedback(ExecuteSpray.Feedback.SPRAYING, 0.60, 'SPRAYING')
            self.get_logger().info(
                f'[ARM][{tree}] SPRAY target={target.target_id} '
                f'duration={request.spray_duration:.1f}s')
            ok, canceled, message = self._spray_target(
                request.mission_id, request.tree_id, request.spray_duration,
                cancel_requested)
            if not ok:
                if canceled:
                    return ExecuteSpray.Result.CANCELED, message
                return self._spray_failure(message, cancel_requested)
            self.get_logger().info(
                f'[ARM][{tree}] SPRAY target={target.target_id} done → TREATED '
                f'({sprayed + 1} sprayed so far)')
            sprayed += 1
            processed.append(target)
            self._select_target('')
            feedback(ExecuteSpray.Feedback.RETURNING_TO_OBSERVE, 0.75,
                     'RETURNING_TO_OBSERVE')
            self.get_logger().info(
                f'[ARM][{tree}] RETURN_TO_OBSERVE distance={self._observation_distance}m')
            if not self._return_to_observation():
                return ExecuteSpray.Result.HOME_FAILED, 'observation return failed'
            self._reset_fruit_tracking()

        for target in self._pending_targets(
                known_targets, processed, exhausted):
            self._mark_unresolved(target, exhausted)
        if sprayed != len(processed) or not target_accounting_is_complete(
                len(known_targets), sprayed, len(exhausted)):
            message = (
                'target accounting invariant failed: '
                f'detected={len(known_targets)} sprayed={sprayed} '
                f'unresolved={len(exhausted)} treated={len(processed)}')
            self.get_logger().error(f'[ARM][{tree}] {message}')
            return self._vision_failure(message, cancel_requested)

        feedback(ExecuteSpray.Feedback.RETURNING_HOME, 0.90, 'RETURNING_HOME')
        self.get_logger().info(f'[ARM][{tree}] HOME returning to home_pose...')
        if not self._return_home(cancel_requested):
            return (ExecuteSpray.Result.CANCELED, 'spray goal canceled') if self._aborted(
                cancel_requested) else (ExecuteSpray.Result.HOME_FAILED, 'HOME motion failed')
        self.get_logger().info(f'[ARM][{tree}] HOME reached')
        summary = spray_summary(
            len(known_targets), sprayed, len(exhausted), alignment_failures,
            recenter_attempts, recenter_failures, alignment_attempts)
        self.get_logger().info(
            f'[ARM][{tree}] ═══ SUMMARY ═══ '
            f'side={request.spray_side} distance={self._observation_distance}m '
            f'{summary}')
        code, message = final_spray_outcome(
            sprayed, len(exhausted), saw_disease, summary)
        if completion_feedback_allowed(code):
            feedback(ExecuteSpray.Feedback.COMPLETED, 1.0, 'COMPLETED')
        return code, message

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

    def _alignment_retry_allowed(self, attempt_count):
        return attempt_count < int(
            self.get_parameter('max_alignment_attempts').value)

    def _rewind_for_untried_observation(self, attempt):
        """Wrap once when a new target starts at the final observation view."""
        current = self._observation_candidate_index
        if current + 1 < len(self._observation_candidates):
            return
        if any(
                index not in attempt.recentered_observation_indices
                for index in range(max(0, current))):
            self._observation_candidate_index = -1

    def _recover_to_next_observation(self, cancel_requested, feedback):
        moved = False
        while not self._aborted(cancel_requested):
            feedback(ExecuteSpray.Feedback.RETURNING_TO_OBSERVE, 0.40,
                     'ALIGN_RECOVERY')
            if not self._move_to_next_observation():
                return False, moved
            moved = True
            self.get_logger().info(
                f'[ARM][ALIGN] moved to recovery observation '
                f'distance={self._observation_distance} m')
            feedback(ExecuteSpray.Feedback.SCANNING_TREE, 0.42,
                     'SCANNING_TREE')
            if self._scan_for_tree(cancel_requested):
                self.get_logger().info(
                    f'[ARM][ALIGN] tree reconfirmed at '
                    f'{self._observation_distance} m')
                return True, moved
            self._set_inference_mode('idle')
            self.get_logger().warn(
                f'[ARM][ALIGN] tree not confirmed at '
                f'{self._observation_distance} m; trying next candidate')
        return False, moved

    def _alignment_recovery_failure(self, message, cancel_requested):
        if self._aborted(cancel_requested):
            return ExecuteSpray.Result.CANCELED, 'spray goal canceled'
        self._set_inference_mode('idle')
        self.get_logger().error(f'[ARM][ALIGN] {message}; returning HOME')
        if self._return_home(cancel_requested):
            return ExecuteSpray.Result.VISION_FAILED, f'{message}; returned HOME'
        locked_message = f'{message}; HOME motion failed; motion locked'
        self.get_logger().error(f'[ARM][ALIGN] {locked_message}')
        self._request_motion_stop()
        return ExecuteSpray.Result.HOME_FAILED, locked_message

    def _scan_for_tree(self, cancel_requested):
        while not self._aborted(cancel_requested):
            self._reset_tree_tracking()
            self._set_inference_mode('tree')
            if self._wait_for_tree(cancel_requested):
                self._publish_observation_debug('tree_confirmed')
                return True
            self._publish_observation_debug(
                'candidate_rejected', rejection_reason='tree_not_confirmed')
            if not self._move_to_next_observation():
                return False
        return False

    def _wait_for_tree(self, cancel_requested):
        deadline = time.monotonic() + float(
            self.get_parameter('scan_pose_detection_timeout_sec').value)
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
        settle = float(
            self.get_parameter('fruit_collection_settle_sec').value)
        required = int(self.get_parameter('confirmation_frames').value)
        ready_since = None
        while time.monotonic() < deadline:
            if self._aborted(cancel_requested):
                return None
            with self._vision_mutex:
                if self._fruit_frames >= required:
                    candidates = [
                        candidate for target_id, candidate in self._fruit_latest.items()
                        if self._fruit_counts.get(target_id, 0) >= required
                    ]
                    if candidates:
                        now = time.monotonic()
                        if ready_since is None:
                            ready_since = now
                        if now - ready_since >= settle:
                            return candidates
            time.sleep(0.02)
        with self._vision_mutex:
            return [] if self._fruit_frames else None

    def _reset_target_confirmation(self, target_id, *, clear_latest=True):
        with self._vision_mutex:
            self._target_confirmation_id = target_id
            self._target_valid_frames = 0
            self._target_confirmation_frames = 0
            self._target_workspace_stable_since = None
            self._target_workspace_last_seen = None
            if clear_latest:
                self._latest_selected_target = None

    def _latest_target(self):
        with self._vision_mutex:
            return self._latest_selected_target

    def _wait_for_target_confirmation(
            self, target_id, cancel_requested, *, require_workspace):
        required = int(self.get_parameter('confirmation_frames').value)
        deadline = time.monotonic() + float(
            self.get_parameter('detection_timeout_sec').value)
        while time.monotonic() < deadline:
            if self._aborted(cancel_requested):
                return False
            with self._vision_mutex:
                if target_id != self._target_confirmation_id:
                    return False
                frames = (
                    self._target_confirmation_frames if require_workspace
                    else self._target_valid_frames)
                stable_duration = (
                    0.0 if self._target_workspace_stable_since is None or
                    self._target_workspace_last_seen is None
                    else max(
                        0.0,
                        self._target_workspace_last_seen -
                        self._target_workspace_stable_since))
                stable_enough = (
                    not require_workspace or
                    stable_duration >= self._recenter_config['post_stable_sec'])
                if frames >= required and stable_enough:
                    return True
            time.sleep(0.02)
        return False

    def _lock_target(self, target_id, cancel_requested):
        """Capture a geometric reference before any camera recenter motion."""
        self._reset_target_confirmation(target_id)
        self._select_target(target_id)
        self._set_inference_mode('target')
        if not self._wait_for_target_confirmation(
                target_id, cancel_requested, require_workspace=False):
            return None
        return self._latest_target()

    def _recenter_target(self, target, attempt, cancel_requested):
        """Move once to a safe camera attitude using the locked mask aim point."""
        inputs = self._wait_for_observation_inputs()
        if inputs is None:
            return False, 'camera or joint state unavailable for target recenter'
        camera, current_joints = inputs
        pre_error_u, pre_error_v = target_pixel_error(
            target.center_u, target.center_v, camera[4], camera[5],
            self._recenter_config['desired_offset_u_px'],
            self._recenter_config['desired_offset_v_px'])
        if not target_requires_recenter(
                target.center_u, target.center_v, camera[4], camera[5],
                self._recenter_config['desired_offset_u_px'],
                self._recenter_config['desired_offset_v_px'],
                self._recenter_config['trigger_px']):
            if not self._wait_for_target_confirmation(
                    target.target_id, cancel_requested, require_workspace=True):
                return False, (
                    'target did not remain reliable inside fine-servo workspace')
            confirmed = self._latest_target()
            post_error_u, post_error_v = target_pixel_error(
                confirmed.center_u, confirmed.center_v, camera[4], camera[5],
                self._recenter_config['desired_offset_u_px'],
                self._recenter_config['desired_offset_v_px'])
            self._publish_observation_debug(
                'target_recenter_not_required',
                target_id=target.target_id,
                pre_error_u_px=pre_error_u,
                pre_error_v_px=pre_error_v,
                post_error_u_px=post_error_u,
                post_error_v_px=post_error_v,
                planned_angle_deg=0.0)
            return True, 'target already inside fine-servo workspace'
        index = self._observation_candidate_index
        if index < 0 or index >= len(self._observation_candidates):
            return False, 'no active observation candidate for target recenter'
        if index in attempt.recentered_observation_indices:
            return False, 'target recenter already used at this observation'
        attempt.recentered_observation_indices.add(index)
        observation = self._observation_candidates[index]
        candidate, angle_deg, rejection_reason = self._move_recenter_step(
            observation, target, camera, current_joints)
        if candidate is None:
            return False, f'target recenter rejected: {rejection_reason}'
        if self._aborted(cancel_requested):
            return False, 'spray goal canceled'
        self._reset_target_confirmation(target.target_id)
        if not self._wait_for_target_confirmation(
                target.target_id, cancel_requested, require_workspace=True):
            self._publish_observation_debug(
                'target_recenter_failed', candidate,
                target_id=target.target_id,
                pre_error_u_px=pre_error_u,
                pre_error_v_px=pre_error_v,
                planned_angle_deg=angle_deg,
                rejection_reason='target_not_reconfirmed')
            return False, 'target was not reconfirmed after recenter'
        confirmed = self._latest_target()
        post_error_u, post_error_v = target_pixel_error(
            confirmed.center_u, confirmed.center_v, camera[4], camera[5],
            self._recenter_config['desired_offset_u_px'],
            self._recenter_config['desired_offset_v_px'])
        total_angle_deg = angle_deg
        iterations = 1
        while (
                iterations < self._recenter_config['max_iterations'] and
                max(abs(post_error_u), abs(post_error_v)) >
                self._recenter_config['refine_goal_px']):
            inputs = self._wait_for_observation_inputs()
            if inputs is None:
                break
            camera, current_joints = inputs
            refined, refine_angle, rejection_reason = self._move_recenter_step(
                candidate, confirmed, camera, current_joints,
                suffix=f'_refine{iterations}')
            if refined is None:
                self._publish_observation_debug(
                    'target_recenter_refine_skipped', candidate,
                    target_id=target.target_id,
                    pre_error_u_px=pre_error_u,
                    pre_error_v_px=pre_error_v,
                    post_error_u_px=post_error_u,
                    post_error_v_px=post_error_v,
                    planned_angle_deg=total_angle_deg,
                    rejection_reason=rejection_reason)
                break
            candidate = refined
            total_angle_deg += refine_angle
            iterations += 1
            self._reset_target_confirmation(target.target_id)
            if not self._wait_for_target_confirmation(
                    target.target_id, cancel_requested,
                    require_workspace=True):
                self._publish_observation_debug(
                    'target_recenter_failed', candidate,
                    target_id=target.target_id,
                    pre_error_u_px=pre_error_u,
                    pre_error_v_px=pre_error_v,
                    planned_angle_deg=total_angle_deg,
                    rejection_reason='target_not_reconfirmed_after_refine')
                return False, 'target was not reconfirmed after recenter refinement'
            confirmed = self._latest_target()
            post_error_u, post_error_v = target_pixel_error(
                confirmed.center_u, confirmed.center_v, camera[4], camera[5],
                self._recenter_config['desired_offset_u_px'],
                self._recenter_config['desired_offset_v_px'])
        self._publish_observation_debug(
            'target_recenter_confirmed', candidate,
            target_id=target.target_id,
            pre_error_u_px=pre_error_u,
            pre_error_v_px=pre_error_v,
            post_error_u_px=post_error_u,
            post_error_v_px=post_error_v,
            planned_angle_deg=total_angle_deg)
        self.get_logger().info(
            f'[ARM][RECENTER] target={target.target_id} '
            f'iterations={iterations} angle={total_angle_deg:.1f}deg '
            f'error=({pre_error_u:.1f},{pre_error_v:.1f})px'
            f'→({post_error_u:.1f},{post_error_v:.1f})px '
            f'condition={candidate.condition_number:.2f} '
            f'joint_margin={candidate.min_joint_margin_rad:.2f}')
        return True, 'target reconfirmed after recenter'

    def _move_recenter_step(
            self, observation, target, camera, current_joints, *, suffix=''):
        rejection_reason = 'no partial recenter candidate was feasible'
        for residual_px in self._recenter_config['residual_candidates_px']:
            try:
                camera_position, camera_quat, angle_deg = recenter_camera_pose(
                    observation.camera_position, observation.camera_quat,
                    camera, target.center_u, target.center_v,
                    self._recenter_config['desired_offset_u_px'],
                    self._recenter_config['desired_offset_v_px'],
                    self._recenter_config['max_angle_deg'],
                    residual_error_px=residual_px)
                tool_position, tool_quat = tool_pose_from_camera_pose(
                    camera_position, camera_quat,
                    self._camera_mount[0], self._camera_mount[1])
            except (TypeError, ValueError) as error:
                rejection_reason = str(error)
                continue
            trial = ObservationCandidate(
                candidate_id=(
                    f'{observation.candidate_id}_target_{target.target_id}'
                    f'_r{residual_px:g}{suffix}'),
                distance_m=observation.distance_m,
                camera_height_m=observation.camera_height_m,
                azimuth_deg=observation.azimuth_deg,
                camera_position=camera_position,
                camera_quat=camera_quat,
                tool_position=tool_position,
                tool_quat=tool_quat,
                visible=True,
                visible_margin_px=math.inf,
            )
            if self._move_to_recentered_pose(trial, current_joints):
                return trial, angle_deg, ''
            rejection_reason = trial.rejection_reason
            if (rejection_reason.startswith('actual_') or
                    rejection_reason in {
                        'moveit_execution_failed',
                        'joint_state_unavailable_after_motion',
                    }):
                break
        return None, 0.0, rejection_reason

    def _move_to_recentered_pose(self, candidate, current_joints):
        """Apply the normal collision IK, singularity and joint-margin gates."""
        ik = self.arm.compute_ik(
            candidate.tool_position, candidate.tool_quat, current_joints)
        if ik is None:
            candidate.rejection_reason = 'collision_ik_failed'
            self._publish_observation_debug('candidate_rejected', candidate)
            return False
        try:
            self._observation_optimizer.evaluate_ik(
                candidate, dict(zip(ik.name, ik.position)), current_joints)
        except (KeyError, TypeError, ValueError):
            candidate.rejection_reason = 'incomplete_ik_state'
        if candidate.rejection_reason:
            self._publish_observation_debug('candidate_rejected', candidate)
            return False
        trajectory = self.arm.plan_pose(
            candidate.tool_position, candidate.tool_quat, frame_id=self._base_frame,
            tolerance_position=self._observation_config['position_tolerance_m'],
            tolerance_orientation=self._observation_config[
                'orientation_tolerance_rad'])
        planned = self.arm.trajectory_final_positions(
            trajectory, self.arm_joint_names) if trajectory is not None else None
        if planned is None:
            candidate.rejection_reason = 'moveit_plan_failed'
            self._publish_observation_debug('candidate_rejected', candidate)
            return False
        self._observation_optimizer.evaluate_ik(candidate, planned, planned)
        if candidate.rejection_reason:
            self._publish_observation_debug('candidate_rejected', candidate)
            return False
        with self._state_mutex:
            joint_state_sequence = self._joint_state_sequence
        if not self.arm.execute_trajectory(trajectory):
            candidate.rejection_reason = 'moveit_execution_failed'
            self._publish_observation_debug('candidate_rejected', candidate)
            return False
        actual = self._wait_for_joint_state(joint_state_sequence)
        if actual is None:
            candidate.rejection_reason = 'joint_state_unavailable_after_motion'
        else:
            self._observation_optimizer.evaluate_ik(candidate, actual, actual)
            if candidate.rejection_reason:
                candidate.rejection_reason = (
                    f'actual_{candidate.rejection_reason}')
        if candidate.rejection_reason:
            self._publish_observation_debug('candidate_rejected', candidate)
            return False
        return True

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
            deduplicate_candidates(kept),
            key=lambda item: (
                math.hypot(
                    item.center_u - float(self.get_parameter('image_width').value) / 2.0,
                    item.center_v - float(self.get_parameter('image_height').value) / 2.0),
                -item.confidence),
        )

    def _remember_targets(self, known, candidates):
        for candidate in candidates:
            for index, previous in enumerate(known):
                if self._same_target(candidate, previous):
                    known[index] = candidate
                    break
            else:
                known.append(candidate)

    def _replace_known_target(self, known, previous, current):
        for index, candidate in enumerate(known):
            if self._same_target(candidate, previous):
                known[index] = current
                known[:] = [
                    candidate for candidate_index, candidate in enumerate(known)
                    if candidate_index == index or
                    not self._same_target(candidate, current)
                ]
                return
        self._remember_targets(known, [current])

    def _pending_targets(self, known, processed, exhausted):
        resolved = processed + exhausted
        return [
            target for target in known
            if not any(self._same_target(target, previous) for previous in resolved)
        ]

    def _attempt_for(self, candidate, attempts):
        return next((attempt for attempt in attempts if self._same_target(
            candidate, attempt.target)), None)

    def _mark_unresolved(self, target, exhausted):
        if not any(self._same_target(target, previous) for previous in exhausted):
            exhausted.append(target)

    def _same_target(self, candidate, previous):
        return (
            candidate.iou(previous) >= float(
                self.get_parameter('processed_iou_threshold').value) or
            candidate.distance_to(previous) <= float(
                self.get_parameter('processed_center_distance_px').value))

    def _reset_vision(self):
        self._observation_pose = None
        self._observation_candidates = []
        self._observation_candidate_index = -1
        self._observation_distance = None
        self._tree_in_base = None
        self._camera_mount = None
        self._reset_fruit_tracking()
        self._reset_tree_tracking()

    def _reset_tree_tracking(self):
        with self._vision_mutex:
            self._tree_frames = 0

    def _reset_fruit_tracking(self):
        with self._vision_mutex:
            self._fruit_frames = 0
            self._fruit_counts = {}
            self._fruit_latest = {}
            self._target_confirmation_id = ''
            self._target_valid_frames = 0
            self._target_confirmation_frames = 0
            self._target_workspace_stable_since = None
            self._target_workspace_last_seen = None
            self._latest_selected_target = None

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
            code = (AlignTarget.Result.CANCELED if canceled
                    else AlignTarget.Result.TIMEOUT)
            return False, canceled, code, error
        result = wrapped.result
        ok = wrapped.status == GoalStatus.STATUS_SUCCEEDED and result.success
        canceled = (
            wrapped.status == GoalStatus.STATUS_CANCELED or
            result.error_code == AlignTarget.Result.CANCELED)
        return (
            ok, canceled, int(result.error_code),
            result.message or f'vision status={wrapped.status}')

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
        self._tree_in_base = tree_in_base
        self._camera_mount = (
            (camera_translation.x, camera_translation.y, camera_translation.z),
            (camera_rotation.x, camera_rotation.y,
             camera_rotation.z, camera_rotation.w),
        )
        if not self._prepare_observation_candidates():
            return False
        return self._move_to_next_observation()

    def _prepare_observation_candidates(self):
        inputs = self._wait_for_observation_inputs()
        if inputs is None:
            self._publish_observation_debug(
                'search_failed', rejection_reason='camera_or_joint_state_unavailable')
            return False
        camera, current_joints = inputs
        started = time.monotonic()
        candidates = self._observation_optimizer.generate(
            self._tree_in_base, self._camera_mount, camera)
        visible_count = sum(candidate.visible for candidate in candidates)
        best_margin = max(
            (candidate.visible_margin_px for candidate in candidates),
            default=-math.inf)
        self.get_logger().info(
            f'[ARM][OBSERVE] tree_in_base=({self._tree_in_base[0]:.2f},'
            f'{self._tree_in_base[1]:.2f},{self._tree_in_base[2]:.2f}) '
            f'camera={camera[4]}x{camera[5]} fx={camera[0]:.1f} fy={camera[1]:.1f} '
            f'generated={len(candidates)} fully_visible={visible_count} '
            f'best_margin_px={best_margin:.1f}')
        for candidate in candidates:
            if not candidate.visible:
                self._publish_observation_debug('candidate_rejected', candidate)
                continue
            if time.monotonic() - started >= float(
                    self.get_parameter('observation_search_timeout_sec').value):
                candidate.rejection_reason = 'ik_search_timeout'
                self._publish_observation_debug('candidate_rejected', candidate)
                continue
            ik = self.arm.compute_ik(
                candidate.tool_position, candidate.tool_quat, current_joints)
            if ik is None:
                candidate.rejection_reason = 'collision_ik_failed'
                self._publish_observation_debug('candidate_rejected', candidate)
                continue
            try:
                self._observation_optimizer.evaluate_ik(
                    candidate, dict(zip(ik.name, ik.position)), current_joints)
            except (KeyError, TypeError, ValueError):
                candidate.rejection_reason = 'incomplete_ik_state'
            self._publish_observation_debug(
                'candidate_ranked' if not candidate.rejection_reason
                else 'candidate_rejected', candidate)
        self._observation_candidates = self._observation_optimizer.rank(candidates)[
            :int(self.get_parameter('observation_max_plans').value)]
        self._observation_candidate_index = -1
        if not self._observation_candidates:
            self._publish_observation_debug(
                'search_failed', rejection_reason='no_servo_safe_candidate')
            return False
        return True

    def _move_to_next_observation(self):
        while self._observation_candidate_index + 1 < len(
                self._observation_candidates):
            self._observation_candidate_index += 1
            candidate = self._observation_candidates[self._observation_candidate_index]
            if self._aborted(lambda: False):
                return False
            trajectory = self.arm.plan_pose(
                candidate.tool_position, candidate.tool_quat, frame_id=self._base_frame,
                tolerance_position=self._observation_config[
                    'position_tolerance_m'],
                tolerance_orientation=self._observation_config[
                    'orientation_tolerance_rad'])
            planned = self.arm.trajectory_final_positions(
                trajectory, self.arm_joint_names) if trajectory is not None else None
            if planned is None:
                candidate.rejection_reason = 'moveit_plan_failed'
                self._publish_observation_debug('candidate_rejected', candidate)
                continue
            self._observation_optimizer.evaluate_ik(candidate, planned, planned)
            if candidate.rejection_reason:
                self._publish_observation_debug('candidate_rejected', candidate)
                continue
            with self._state_mutex:
                joint_state_sequence = self._joint_state_sequence
            if self.arm.execute_trajectory(trajectory):
                actual = self._wait_for_joint_state(joint_state_sequence)
                if actual is None:
                    candidate.rejection_reason = 'joint_state_unavailable_after_motion'
                else:
                    self._observation_optimizer.evaluate_ik(candidate, actual, actual)
                if candidate.rejection_reason:
                    self._publish_observation_debug('candidate_rejected', candidate)
                    continue
                self._observation_distance = candidate.distance_m
                self._observation_pose = (candidate.tool_position, candidate.tool_quat)
                self.get_logger().info(
                    f'[ARM][ALIGN] selected observation candidate '
                    f'index={self._observation_candidate_index} '
                    f'id={candidate.candidate_id} distance={candidate.distance_m} m '
                    f'condition={candidate.condition_number:.2f} '
                    f'joint_margin={candidate.min_joint_margin_rad:.2f}')
                self._publish_observation_debug('candidate_selected', candidate)
                return True
            candidate.rejection_reason = 'moveit_execution_failed'
            self._publish_observation_debug('candidate_rejected', candidate)
            self.get_logger().warn(
                f'[ARM][ALIGN] planning failed for observation candidate '
                f'index={self._observation_candidate_index} '
                f'id={candidate.candidate_id}')
        return False

    def _wait_for_observation_inputs(self):
        deadline = time.monotonic() + float(
            self.get_parameter('observation_input_timeout_sec').value)
        while time.monotonic() < deadline:
            with self._state_mutex:
                camera = self._camera_model
                joints = self._joint_positions
            if camera is not None and joints is not None:
                return camera, joints
            time.sleep(0.02)
        return None

    def _wait_for_joint_state(self, after_sequence=None):
        deadline = time.monotonic() + float(
            self.get_parameter('observation_input_timeout_sec').value)
        while time.monotonic() < deadline:
            with self._state_mutex:
                joints = self._joint_positions
                sequence = self._joint_state_sequence
            if joints is not None and (
                    after_sequence is None or sequence > after_sequence):
                return joints
            time.sleep(0.02)
        return None

    def _publish_observation_debug(
            self, event, candidate=None, rejection_reason='', *, target_id='',
            pre_error_u_px=None, pre_error_v_px=None, post_error_u_px=None,
            post_error_v_px=None, planned_angle_deg=None):
        if candidate is None:
            candidate_id = ''
            distance = camera_height = azimuth = 0.0
            visible = ik_valid = selected = False
            condition = math.inf
            margin = motion = 0.0
        else:
            candidate_id = candidate.candidate_id
            distance = candidate.distance_m
            camera_height = candidate.camera_height_m
            azimuth = candidate.azimuth_deg
            visible = bool(candidate.visible)
            ik_valid = candidate.ik_joints is not None
            selected = event == 'candidate_selected'
            condition = candidate.condition_number
            margin = candidate.min_joint_margin_rad
            motion = candidate.joint_motion_norm
            rejection_reason = rejection_reason or candidate.rejection_reason
        payload = {
            'event': event,
            'mission_id': self._active_mission,
            'tree_id': self._active_tree,
            'candidate_id': candidate_id,
            'distance_m': distance,
            'camera_height_m': camera_height,
            'azimuth_deg': azimuth,
            'visible': visible,
            'ik_valid': ik_valid,
            'condition_number': None if not math.isfinite(condition) else condition,
            'min_joint_margin_rad': margin,
            'joint_motion_norm': motion,
            'rejection_reason': rejection_reason,
            'selected': selected,
            'target_id': target_id,
            'pre_error_u_px': pre_error_u_px,
            'pre_error_v_px': pre_error_v_px,
            'post_error_u_px': post_error_u_px,
            'post_error_v_px': post_error_v_px,
            'planned_angle_deg': planned_angle_deg,
        }
        self._observation_debug_pub.publish(String(data=json.dumps(
            payload, sort_keys=True, separators=(',', ':'))))
        if event != 'candidate_rejected' or visible:
            self.get_logger().info(
                f'[ARM][OBSERVE] event={event} id={candidate_id or "-"} '
                f'visible={visible} ik={ik_valid} '
                f'condition={payload["condition_number"]} '
                f'joint_margin={margin:.3f} reason={rejection_reason or "-"}')

    def _return_to_observation(self):
        if self._observation_pose is None or self._abort.is_set():
            return False
        position, quat = self._observation_pose
        return self._move_to_pose((position, quat))

    def _move_to_pose(self, pose):
        position, quat = pose
        return self.arm.move_pose(
            position, quat, frame_id=self._base_frame,
            tolerance_position=self._observation_config[
                'position_tolerance_m'],
            tolerance_orientation=self._observation_config[
                'orientation_tolerance_rad'])

    def _return_home(self, cancel_requested):
        return not self._aborted(cancel_requested) and self.arm.move_joints(self._home)

    def _select_target(self, target_id):
        message = String()
        message.data = target_id
        self._selected_target_pub.publish(message)

    def _set_inference_mode(self, mode):
        message = String()
        message.data = mode
        self._inference_mode_pub.publish(message)

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
