"""ROS task node for the Alicia-M orchard spraying workflow."""

import math
import threading

from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data)
from sensor_msgs.msg import CameraInfo, JointState
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener
from vision_msgs.msg import Detection2DArray
from wvcsc_interfaces.action import AlignTarget, ExecuteSpray, Spray
from wvcsc_interfaces.msg import Target2D

from .action_flow import DownstreamActionMixin
from .motion_state import MotionControlState
from .node_parameters import create_alicia_moveit
from .observation import ObservationFlowMixin, ObservationOptimizer
from .target_flow import (TargetAttempt, TargetFlowMixin,
                          completion_feedback_allowed, final_spray_outcome,
                          spray_summary, target_accounting_is_complete)

class SprayTask(TargetFlowMixin, ObservationFlowMixin, DownstreamActionMixin, Node):
    """协调 MoveIt、YOLO、视觉伺服和喷洒执行器的长时 Action Server。

    订阅回调只更新受互斥锁保护的最新视觉/关节快照，Action 执行线程运行状态机；
    ``MotionControlState`` 的锁定和 cancel epoch 始终优先于任务推进。失败恢复只会
    尝试已筛选的下一观察位，不能通过降低碰撞、奇异点或关节余量阈值强行执行。
    """

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
        self._target_workspace_anchor = None
        self._target_workspace_currently_valid = False
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
            # MoveIt owns coarse placement only outside the 48 px IBVS work
            # area; continuous Servo owns final sub-pixel visual correction.
            'target_recenter_trigger_px': 48.0,
            'target_recenter_workspace_px': 48.0,
            'target_recenter_max_angle_deg': 20.0,
            'target_recenter_refine_goal_px': 24.0,
            'target_recenter_max_iterations': 2,
            'target_recenter_residual_candidates_px': [
                12.0, 16.0, 24.0, 8.0, 32.0, 40.0, 3.0, 1.0, 0.0],
            # Observation search permits centimetre-level placement. The
            # target recenter needs a tighter endpoint to meet <2 px.
            'target_recenter_position_tolerance_m': 0.002,
            'target_recenter_orientation_tolerance_rad': 0.002,
            # Must match wvcsc_visual_servo/config/visual_servo.yaml.
            'target_recenter_desired_offset_u_px': 0.0,
            'target_recenter_desired_offset_v_px': 28.0,
            'target_post_recenter_stable_sec': 0.20,
            'target_post_recenter_max_drift_px': 4.0,
            'target_post_recenter_max_gap_sec': 0.20,
            'target_post_recenter_min_confidence': 0.30,
            # After the last known target is treated, two independent empty
            # observations are used to avoid silently accepting a fruit that
            # is missed by the first view. A larger value is safer for deeply
            # occluded real trees, but adds a full MoveIt trajectory per view.
            'completion_scan_empty_limit': 2,
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
            'workspace_px': float(
                self.get_parameter('target_recenter_workspace_px').value),
            'max_angle_deg': float(self.get_parameter('target_recenter_max_angle_deg').value),
            'refine_goal_px': float(
                self.get_parameter('target_recenter_refine_goal_px').value),
            'max_iterations': int(
                self.get_parameter('target_recenter_max_iterations').value),
            'residual_candidates_px': tuple(dict.fromkeys(
                float(value) for value in self.get_parameter(
                    'target_recenter_residual_candidates_px').value)),
            'position_tolerance_m': float(self.get_parameter(
                'target_recenter_position_tolerance_m').value),
            'orientation_tolerance_rad': float(self.get_parameter(
                'target_recenter_orientation_tolerance_rad').value),
            'desired_offset_u_px': float(
                self.get_parameter('target_recenter_desired_offset_u_px').value),
            'desired_offset_v_px': float(
                self.get_parameter('target_recenter_desired_offset_v_px').value),
            'post_stable_sec': float(
                self.get_parameter('target_post_recenter_stable_sec').value),
            'post_max_drift_px': float(self.get_parameter(
                'target_post_recenter_max_drift_px').value),
            'post_max_gap_sec': float(self.get_parameter(
                'target_post_recenter_max_gap_sec').value),
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
                values['workspace_px'] < values['trigger_px'] or
                values['refine_goal_px'] <= 0.0 or
                values['refine_goal_px'] > values['trigger_px'] or
                values['position_tolerance_m'] <= 0.0 or
                values['orientation_tolerance_rad'] <= 0.0 or
                not 1 <= values['max_iterations'] <= 5 or
                not values['residual_candidates_px'] or
                any(not math.isfinite(value) or value < 0.0 or
                    value >= values['workspace_px']
                    for value in values['residual_candidates_px']) or
                values['post_stable_sec'] <= 0.0 or
                values['post_max_drift_px'] <= 0.0 or
                values['post_max_gap_sec'] <= 0.0 or
                values['post_max_gap_sec'] > values['post_stable_sec'] or
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
        """执行一棵树的完整闭环，并返回 ``ExecuteSpray`` 结果码和摘要。

        每次喷洒后回到观察位重新检测，避免机械臂运动导致旧图像坐标失效。目标集合
        按几何关系跨轮合并；循环退出前强制满足
        ``detected == sprayed + unresolved``，从而禁止病果静默丢失。
        """
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
            return self._recover_failure(
                ExecuteSpray.Result.OBSERVE_FAILED,
                'observation motion failed', cancel_requested,
                'observation and HOME motion failed')
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
            return self._recover_failure(
                ExecuteSpray.Result.VISION_FAILED,
                'tree was not confirmed in the camera view', cancel_requested)
        self.get_logger().info(f'[ARM][{tree}] SCAN tree confirmed')

        processed = []
        exhausted = []
        known_targets = []
        attempts = []
        surveyed_observation_indices = set()
        pending_attempt = None
        sprayed = 0
        saw_disease = False
        alignment_failures = 0
        recenter_attempts = 0
        recenter_failures = 0
        alignment_attempts = 0
        completion_scan_empty_count = 0
        while True:
            self._set_inference_mode('fruits')
            feedback(ExecuteSpray.Feedback.DETECTING_FRUITS, 0.25, 'DETECTING_FRUITS')
            self.get_logger().debug(
                f'[ARM][{tree}] DETECT inference_mode=fruits '
                f'timeout={float(self.get_parameter("detection_timeout_sec").value):.1f}s '
                f'confirmation={int(self.get_parameter("confirmation_frames").value)}')
            candidates = self._wait_for_fruits(cancel_requested)
            if candidates is None:
                return self._recover_failure(
                    ExecuteSpray.Result.VISION_FAILED,
                    'fruit detector did not provide frames', cancel_requested)
            if self._observation_candidate_index >= 0:
                surveyed_observation_indices.add(
                    self._observation_candidate_index)
            saw_disease = saw_disease or bool(candidates)
            feedback(ExecuteSpray.Feedback.QUEUING, 0.35, 'QUEUING')
            queue = self._queue(candidates, processed + exhausted)
            if queue:
                # A non-empty completion view found another target; require a
                # fresh empty-view confirmation after that target is treated.
                completion_scan_empty_count = 0
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
                f'[ARM][{tree}] DETECT_QUEUE candidates={len(candidates)} '
                f'ids=({",".join(c.target_id for c in candidates[:8])})'
                f'{"..." if len(candidates) > 8 else ""} '
                f'processed={len(processed)} exhausted={len(exhausted)} '
                f'queued={len(queue)}')
            if not queue:
                pending_targets = self._pending_targets(
                    known_targets, processed, exhausted)
                missing_target = (
                    pending_attempt.target if pending_attempt is not None
                    else (pending_targets[0] if pending_targets else None))
                if missing_target is not None:
                    (attempt, recovered, _moved) = self._recover_missing_target(
                        missing_target, pending_attempt, attempts,
                        cancel_requested, feedback)
                    if recovered:
                        pending_attempt = attempt
                        continue
                if pending_attempt is not None:
                    self._mark_unresolved(pending_attempt.target, exhausted)
                    self.get_logger().warn(
                        f'[ARM][ALIGN] target={pending_attempt.target.target_id} '
                        'was not redetected after exhausting safe observation views; '
                        'marked unresolved')
                    pending_attempt = None
                completion_scan_limit = int(self.get_parameter(
                    'completion_scan_empty_limit').value)
                if completion_scan_empty_count >= completion_scan_limit:
                    self.get_logger().info(
                        f'[ARM][{tree}] completion scan empty limit reached '
                        f'({completion_scan_empty_count}/{completion_scan_limit})')
                    break
                for target in self._pending_targets(
                        known_targets, processed, exhausted):
                    self._mark_unresolved(target, exhausted)
                    self.get_logger().warn(
                        f'[ARM][QUEUE] target={target.target_id} disappeared '
                        'after exhausting safe observation views; marked unresolved')
                recovered, moved = self._recover_for_completion_scan(
                    surveyed_observation_indices, cancel_requested, feedback)
                if recovered:
                    completion_scan_empty_count += 1
                    pending_attempt = None
                    continue
                if moved and self._aborted(cancel_requested):
                    return ExecuteSpray.Result.CANCELED, 'spray goal canceled'
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
                if self._aborted(cancel_requested):
                    return ExecuteSpray.Result.CANCELED, 'spray goal canceled'
                self._select_target('')
                self._set_inference_mode('idle')
                self.get_logger().warn(
                    f'[ARM][RECENTER] target={target.target_id} {recenter_message}; '
                    'trying the next observation candidate')
                self._rewind_for_untried_observation(attempt)
                recovered, moved = self._recover_to_next_observation(
                    cancel_requested, feedback,
                    attempt.recentered_observation_indices)
                if recovered:
                    pending_attempt = attempt
                    self._reset_fruit_tracking()
                    continue
                if moved:
                    # A rejected candidate that is recovered at another safe
                    # observation is a fallback, not a failed tree target.
                    # Count only unrecoverable recenter failures in SUMMARY.
                    recenter_failures += 1
                    return self._alignment_recovery_failure(
                        f'target recenter failed: {recenter_message}; '
                        'tree reconfirmation failed after observation recovery',
                        cancel_requested)
                recenter_failures += 1
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
                f'[ARM][ALIGN] ENTER_VISUAL_SERVO target={target.target_id} '
                f'attempt={attempt.count}/'
                f'{int(self.get_parameter("max_alignment_attempts").value)} '
                f'timeout={self._vision_timeout:.1f}s '
                f'observation_distance={self._observation_distance}')
            ok, canceled, align_code, message = self._align_target(
                request.mission_id, request.tree_id, target.target_id, cancel_requested)
            self.get_logger().debug(
                f'[ARM][ALIGN] result target={target.target_id} '
                f'code={align_code} message={message}')
            if not ok:
                alignment_failures += 1
                if canceled:
                    return ExecuteSpray.Result.CANCELED, message
                recoverable = self._is_recoverable_alignment_code(align_code)
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
                        cancel_requested, feedback,
                        attempt.recentered_observation_indices)
                    if recovered:
                        pending_attempt = attempt
                        self._reset_fruit_tracking()
                        self.get_logger().info(
                            f'[ARM][ALIGN] recovery ready at '
                            f'{self._observation_distance} m; redetecting fruit')
                        continue
                    if moved:
                        recenter_failures += 1
                        return self._alignment_recovery_failure(
                            f'visual alignment code={align_code}: {message}; '
                            'tree reconfirmation failed after observation recovery',
                            cancel_requested)
                recenter_failures += 1
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
                return self._recover_failure(
                    ExecuteSpray.Result.SPRAY_FAILED, message, cancel_requested)
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
            return self._recover_failure(
                ExecuteSpray.Result.VISION_FAILED, message, cancel_requested)

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

    def _recover_failure(
            self, result_code, message, cancel_requested,
            home_failure_message=None):
        """Return a task failure only after attempting the mandatory HOME recovery."""
        if self._aborted(cancel_requested):
            return ExecuteSpray.Result.CANCELED, 'spray goal canceled'
        if not self._return_home(cancel_requested):
            return ExecuteSpray.Result.HOME_FAILED, (
                home_failure_message or f'{message}; HOME motion failed')
        return result_code, message

    def _alignment_retry_allowed(self, attempt_count):
        return attempt_count < int(
            self.get_parameter('max_alignment_attempts').value)

    @staticmethod
    def _is_recoverable_alignment_code(align_code):
        return align_code in {
            AlignTarget.Result.TIMEOUT,
            AlignTarget.Result.TARGET_STALE,
            AlignTarget.Result.SERVO_SINGULARITY,
            AlignTarget.Result.SERVO_SAFETY_STOP,
        }

    def _rewind_for_untried_observation(self, attempt):
        """Wrap once when a new target starts at the final observation view."""
        current = self._observation_candidate_index
        if current + 1 < len(self._observation_candidates):
            return
        if any(
                index not in attempt.recentered_observation_indices
                for index in range(max(0, current))):
            self._observation_candidate_index = -1

    def _recover_missing_target(
            self, target, pending_attempt, attempts,
            cancel_requested, feedback):
        """在判定病果 unresolved 前，到尚未搜索的安全观察位重新检测。

        单轮 YOLO 检测为空只能说明当前视角暂时漏检，不能证明病果已经消失。复用
        现有观察候选和树木确认流程，使目标在所有安全视角耗尽后才进入
        ``UNRESOLVED``。当前视角加入目标的已尝试集合，保证搜索有界。
        """
        attempt = pending_attempt or self._attempt_for(target, attempts)
        if attempt is None:
            attempt = TargetAttempt(target)
            attempts.append(attempt)
        current = self._observation_candidate_index
        if current >= 0:
            attempt.recentered_observation_indices.add(current)
        self._select_target('')
        self._set_inference_mode('idle')
        self._rewind_for_untried_observation(attempt)
        recovered, moved = self._recover_to_next_observation(
            cancel_requested, feedback,
            attempt.recentered_observation_indices)
        if recovered:
            self._reset_fruit_tracking()
            self.get_logger().info(
                f'[ARM][QUEUE] target={target.target_id} missing in current view; '
                f'retrying detection at observation '
                f'index={self._observation_candidate_index}')
        return attempt, recovered, moved

    def _recover_for_completion_scan(
            self, surveyed_indices, cancel_requested, feedback):
        """Search one unvisited safe view before declaring the tree complete.

        A queue empty in one image does not prove that every diseased fruit is
        visible: branches can occlude another fruit from the current azimuth.
        The same collision-checked observation list used for target recovery is
        therefore exhausted once at tree completion.  Previously surveyed views
        are excluded, keeping the search finite and avoiding repeated motions.
        """
        if len(surveyed_indices) >= len(self._observation_candidates):
            return False, False
        current = self._observation_candidate_index
        if (current + 1 >= len(self._observation_candidates) and
                any(index not in surveyed_indices
                    for index in range(len(self._observation_candidates)))):
            self._observation_candidate_index = -1
        self._select_target('')
        self._set_inference_mode('idle')
        recovered, moved = self._recover_to_next_observation(
            cancel_requested, feedback, surveyed_indices)
        if recovered:
            self._reset_fruit_tracking()
            self.get_logger().info(
                f'[ARM][QUEUE] completion scan at unvisited observation '
                f'index={self._observation_candidate_index}')
        return recovered, moved

    def _recover_to_next_observation(
            self, cancel_requested, feedback, excluded_indices=None):
        """移至该病果尚未尝试过的下一个安全观察位。

        ``TargetAttempt`` 记录每个已经执行过重心的观察位。恢复搜索发生回绕时必须
        跳过这些索引，否则会在同一姿态再次得到 ``recenter already used``，既浪费
        轨迹时间，也会提前耗尽恢复流程。
        """
        moved = False
        while not self._aborted(cancel_requested):
            feedback(ExecuteSpray.Feedback.RETURNING_TO_OBSERVE, 0.40,
                     'ALIGN_RECOVERY')
            if not self._move_to_next_observation(excluded_indices):
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


import rclpy
from rclpy.executors import MultiThreadedExecutor

__all__ = ['SprayTask']


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
