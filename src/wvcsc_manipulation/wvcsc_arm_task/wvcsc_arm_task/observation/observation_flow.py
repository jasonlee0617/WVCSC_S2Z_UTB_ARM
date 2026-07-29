"""ROS 观察流程与共享候选执行。

IK 几何和运动学实现位于 :mod:`ik_observation`，固定关节扫描策略位于
:mod:`joint_preset_observation`。本模块只负责 ROS 快照、候选执行、
目标重心、观察位切换和调试发布。
"""

import math
import time

import rclpy
from tf2_ros import TransformException

from .ik_observation import (
    ObservationOptimizer,
    camera_orientation_for_pixel,
    nozzle_pose_from_tool_pose,
    nozzle_tree_plane_metrics,
    recenter_camera_pose,
    tool_pose_from_camera_pose,
    transform_point,
)
from .candidate import ObservationCandidate, build_candidate
from ..target_flow import target_pixel_error


class ObservationFlowMixin:
    def _move_to_next_fan_observation(self):
        """Advance center -> left fan -> right fan, never into recovery views."""
        next_index = self._observation_candidate_index + 1
        if next_index >= len(self._observation_candidates):
            return False
        phase = getattr(
            self._observation_candidates[next_index], 'selection_phase', '')
        if not str(phase).startswith('fan_'):
            return False
        return self._move_to_next_observation()

    def _active_observation_candidate(self):
        index = self._observation_candidate_index
        if 0 <= index < len(self._observation_candidates):
            return self._observation_candidates[index]
        return None

    # --------- 目标重心与姿态修正 ---------
    def _motion_preflight(
            self, target, current_joints, *, source, error_norm_px, stage):
        """Verify the stationary state before a safety-critical next step.

        Collision validity comes from the successful MoveIt observation or
        recenter plan that placed the arm at this state.  Re-solving IK for the
        same pose could choose a different branch and would not prove that the
        measured current state is collision-free.  The missing stationary
        checks are therefore the Jacobian condition number and joint-limit
        margin, evaluated from the actual joint feedback.
        """
        stage_message = 'Servo' if stage == 'SERVO' else stage
        index = self._observation_candidate_index
        if index < 0 or index >= len(self._observation_candidates):
            return False, (
                f'no active observation candidate for {stage_message} preflight')
        observation = self._observation_candidates[index]
        if observation.rejection_reason:
            return False, (
                'active observation is not motion-safe: '
                f'{observation.rejection_reason}')
        preflight = build_candidate(
            candidate_id=f'{observation.candidate_id}_servo_preflight',
            distance_m=observation.distance_m,
            camera_height_m=observation.camera_height_m,
            azimuth_deg=observation.azimuth_deg,
            camera_position=observation.camera_position,
            camera_quat=observation.camera_quat,
            tool_position=observation.tool_position,
            tool_quat=observation.tool_quat,
            visible=True,
            visible_margin_px=math.inf,
            observation_mode=getattr(observation, 'observation_mode', 'ik'),
            joint_positions=getattr(observation, 'joint_positions', ()),
        )
        try:
            self._observation_optimizer.evaluate_ik(
                preflight, current_joints, current_joints)
        except (KeyError, TypeError, ValueError):
            return False, f'incomplete joint state for {stage_message} preflight'
        if preflight.rejection_reason:
            return False, (
                f'{stage_message} preflight rejected: '
                f'{preflight.rejection_reason}')
        self.get_logger().info(
            f'[ARM][{stage}_PREFLIGHT] target={target.target_id} '
            f'mode={getattr(observation, "observation_mode", "ik")} '
            f'source={source} error={error_norm_px:.1f}px '
            f'condition={preflight.condition_number:.2f} '
            f'joint_margin={preflight.min_joint_margin_rad:.2f} '
            'collision=active_moveit_plan')
        return True, ''

    def _servo_handoff_preflight(
            self, target, current_joints, *, source, error_norm_px):
        """Verify the stationary observation state before starting Servo."""
        return self._motion_preflight(
            target, current_joints, source=source,
            error_norm_px=error_norm_px, stage='SERVO')

    def _confirm_servo_handoff(
            self, target, current_joints, desired_u, desired_v,
            cancel_requested, *, pre_error_u, pre_error_v, source,
            confirmed=None):
        """Reconfirm a target, enforce the bounded Servo entry, then preflight."""
        if confirmed is None:
            if not self._wait_for_target_confirmation(
                    target.target_id, cancel_requested, require_workspace=False):
                return False, 'target was not freshly reconfirmed before visual servo'
            confirmed = self._latest_target()
        post_error_u, post_error_v = target_pixel_error(
            confirmed.center_u, confirmed.center_v, desired_u, desired_v)
        post_error_norm = math.hypot(post_error_u, post_error_v)
        entry_px = self._recenter_config['servo_entry_px']
        if post_error_norm > entry_px:
            return False, (
                f'target residual {post_error_norm:.1f}px exceeds Servo entry '
                f'tolerance {entry_px:.1f}px')
        inputs = self._wait_for_observation_inputs()
        if inputs is None:
            return False, 'camera or joint state unavailable for Servo preflight'
        _camera, current_joints = inputs
        preflight_ok, preflight_message = self._servo_handoff_preflight(
            confirmed, current_joints, source=source,
            error_norm_px=post_error_norm)
        if not preflight_ok:
            return False, preflight_message
        return True, 'target reconfirmed for visual servo'

    def _recenter_target(self, target, attempt, cancel_requested):
        """使用掩膜安全瞄准点执行有限角度重心，并重新确认同一逻辑目标。

        重心只旋转相机姿态、保持位置和喷洒距离；每步都重新做碰撞 IK、条件数和
        关节余量检查。目标丢失、关联歧义或安全筛选失败时返回可恢复失败，由上层
        切换观察候选，绝不就近改选另一颗病果。
        """
        inputs = self._wait_for_observation_inputs()
        if inputs is None:
            return False, 'camera or joint state unavailable for target recenter'
        camera, current_joints = inputs
        desired_aim = self._active_aim_pixel(camera[4], camera[5])
        if desired_aim is None:
            return False, 'calibrated nozzle aim is unavailable for target recenter'
        desired_u, desired_v = desired_aim
        pre_error_u, pre_error_v = target_pixel_error(
            target.center_u, target.center_v, desired_u, desired_v)
        
        trigger_px = self._recenter_config['trigger_px']
        pre_error_norm = math.hypot(pre_error_u, pre_error_v)
        if pre_error_norm <= trigger_px:
            return self._confirm_servo_handoff(
                target, current_joints, desired_u, desired_v, cancel_requested,
                pre_error_u=pre_error_u, pre_error_v=pre_error_v,
                source='inside_recenter_trigger')
        # 目标偏离多少像素不再是重心准入条件。真正的物理边界由单步/累计
        # 旋转角、碰撞 IK、关节余量及 MoveIt 规划共同决定；固定 128 px
        # 门限曾错误拒绝约 19°、但仍在 20°安全重心范围内的病果。
        index = self._observation_candidate_index
        if index < 0 or index >= len(self._observation_candidates):
            return False, 'no active observation candidate for target recenter'
        if index in attempt.recentered_observation_indices:
            return False, 'target recenter already used at this observation'
        attempt.recentered_observation_indices.add(index)
        observation = self._observation_candidates[index]
        
        # 获取真实的相机位姿作为初始起点（不是规划的终点）
        camera_pose = self._current_camera_pose()
        if camera_pose is None:
            return False, 'actual camera pose unavailable for target recenter'
        maximum_total_angle = self._recenter_config.get(
            'max_total_angle_deg', math.inf)
        candidate, angle_deg, rejection_reason = self._move_recenter_step(
            observation, target, camera, current_joints, camera_pose=camera_pose,
            max_angle_deg=min(
                self._recenter_config['max_angle_deg'], maximum_total_angle))
        if candidate is None:
            return False, f'target recenter rejected: {rejection_reason}'
        if self._aborted(cancel_requested):
            return False, 'spray goal canceled'
        self._reset_target_confirmation(target.target_id)
        if not self._wait_for_target_confirmation(
                target.target_id, cancel_requested, require_workspace=False):
            return False, 'target was not reconfirmed after recenter'
        
        confirmed = self._latest_target()
        post_error_u, post_error_v = target_pixel_error(
            confirmed.center_u, confirmed.center_v, desired_u, desired_v)
        total_angle_deg = angle_deg
        iterations = 1
        self.get_logger().info(
            f'[ARM][RECENTER_STEP] target={target.target_id} iteration=1 '
            f'angle={angle_deg:.1f}deg total={total_angle_deg:.1f}deg '
            f'error={math.hypot(pre_error_u, pre_error_v):.1f}px'
            f'→{math.hypot(post_error_u, post_error_v):.1f}px')
        
        # 循环细化重心，直到误差小于 refine_goal_px 或达到最大迭代次数
        while (
                iterations < self._recenter_config['max_iterations'] and
                total_angle_deg < maximum_total_angle and
                math.hypot(post_error_u, post_error_v) >
                self._recenter_config['refine_goal_px']):
            inputs = self._wait_for_observation_inputs()
            if inputs is None:
                break
            camera, current_joints = inputs
            desired_aim = self._active_aim_pixel(camera[4], camera[5])
            if desired_aim is None:
                break
            desired_u, desired_v = desired_aim
            camera_pose = self._current_camera_pose()
            if camera_pose is None:
                break
            refined, refine_angle, rejection_reason = self._move_recenter_step(
                candidate, confirmed, camera, current_joints, camera_pose=camera_pose,
                suffix=f'_refine{iterations}', max_angle_deg=min(
                    self._recenter_config['max_angle_deg'],
                    maximum_total_angle - total_angle_deg))
            if refined is None:
                self.get_logger().warn(
                    f'[ARM][RECENTER_STEP] target={target.target_id} '
                    f'iteration={iterations + 1} rejected '
                    f'error={math.hypot(post_error_u, post_error_v):.1f}px '
                    f'reason={rejection_reason}')
                break
            candidate = refined
            total_angle_deg += refine_angle
            iterations += 1
            self._reset_target_confirmation(target.target_id)
            if not self._wait_for_target_confirmation(
                    target.target_id, cancel_requested,
                    require_workspace=False):
                return False, 'target was not reconfirmed after recenter refinement'
            confirmed = self._latest_target()
            previous_error_norm = math.hypot(post_error_u, post_error_v)
            post_error_u, post_error_v = target_pixel_error(
                confirmed.center_u, confirmed.center_v, desired_u, desired_v)
            self.get_logger().info(
                f'[ARM][RECENTER_STEP] target={target.target_id} '
                f'iteration={iterations} angle={refine_angle:.1f}deg '
                f'total={total_angle_deg:.1f}deg '
                f'error={previous_error_norm:.1f}px'
                f'→{math.hypot(post_error_u, post_error_v):.1f}px')
        # 粗对准只负责把一个新鲜、有效的锁定目标送入 Servo 可控窗口；最终
        # 4px/0.5s 稳定性仍由 AlignTarget 闭环统一判定。
        handoff_ok, handoff_message = self._confirm_servo_handoff(
            target, current_joints, desired_u, desired_v, cancel_requested,
            pre_error_u=pre_error_u, pre_error_v=pre_error_v,
            source='safe_moveit_recenter', confirmed=confirmed)
        if not handoff_ok:
            return False, handoff_message
        self.get_logger().info(
            f'[ARM][RECENTER] target={target.target_id} '
            f'iterations={iterations} angle={total_angle_deg:.1f}deg '
            f'error=({pre_error_u:.1f},{pre_error_v:.1f})px'
            f'→({post_error_u:.1f},{post_error_v:.1f})px '
            f'condition={candidate.condition_number:.2f} '
            f'joint_margin={candidate.min_joint_margin_rad:.2f}')
        return True, 'target reconfirmed after recenter'

    def _move_recenter_step(
            self, observation, target, camera, current_joints, *, camera_pose=None,
            suffix='', max_angle_deg=None, residual_candidates=None):
        """用真实 C10 起点生成并执行一次安全的重心姿态。

        ``observation`` 只提供候选的距离、高度、方位和诊断身份；若调用方传入
        ``camera_pose``，它来自 ``base -> camera_color_optical_frame`` 的最新 TF。
        这样多次重心会根据真实执行终点继续修正，而不是在旧的计划终点附近重复
        计算同一个旋转。没有显式位姿时保留候选中的几何值，方便纯单元测试。
        """
        if camera_pose is None:
            start_camera_position = observation.camera_position
            start_camera_quat = observation.camera_quat
        else:
            start_camera_position, start_camera_quat = camera_pose
        rejection_reason = 'no partial recenter candidate was feasible'
        desired_aim = self._active_aim_pixel(camera[4], camera[5])
        if desired_aim is None:
            return None, 0.0, 'calibrated nozzle aim is unavailable'
        if max_angle_deg is None:
            max_angle_deg = self._recenter_config['max_angle_deg']
        if max_angle_deg <= 0.0:
            return None, 0.0, 'recenter total angle limit reached'
        # 闭环模式尝试多个残差；实机单次对准显式传入 (0.0,)。
        residual_candidates = (
            self._recenter_config['residual_candidates_px']
            if residual_candidates is None else tuple(residual_candidates))
        if not residual_candidates:
            return None, 0.0, 'no recenter residual candidates configured'
        for residual_px in residual_candidates:
            try:
                camera_position, camera_quat, angle_deg = recenter_camera_pose(
                    start_camera_position, start_camera_quat,
                    camera, target.center_u, target.center_v,
                    *desired_aim,
                    max_angle_deg,
                    residual_error_px=residual_px)
                tool_position, tool_quat = tool_pose_from_camera_pose(
                    camera_position, camera_quat,
                    self._camera_mount[0], self._camera_mount[1])
            except (TypeError, ValueError) as error:
                rejection_reason = str(error)
                continue
            trial = build_candidate(
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

    def _execute_candidate_motion(
            self, candidate, *, current_joints=None,
            tolerance_position=None, tolerance_orientation=None,
            validate_target_ik=False, prefix_actual_rejection=False):
        """执行候选位姿的统一安全流程。

        普通观察候选在生成阶段已经完成碰撞 IK，重心候选则必须在执行前重新
        计算 IK，因此通过 ``validate_target_ik`` 保持两条原有路径的语义差异。
        两条路径共享计划轨迹、执行、最新关节状态和实际安全指标复核逻辑。
        """
        # 1. 如果通过 IK 校验执行重心步骤，则计算当前状态下的 IK
        if validate_target_ik:
            ik = self.arm.compute_ik(
                candidate.tool_position, candidate.tool_quat, current_joints)
            if ik is None:
                candidate.rejection_reason = 'collision_ik_failed'
                return False
            try:
                self._observation_optimizer.evaluate_ik(
                    candidate, dict(zip(ik.name, ik.position)), current_joints)
            except (KeyError, TypeError, ValueError):
                candidate.rejection_reason = 'incomplete_ik_state'
            if candidate.rejection_reason:
                return False

        # 2. 设置姿态公差并规划轨迹
        if tolerance_position is None or tolerance_orientation is None:
            tolerance_position = self._recenter_config['position_tolerance_m']
            tolerance_orientation = self._recenter_config[
                'orientation_tolerance_rad']
        trajectory = self.arm.plan_pose(
            candidate.tool_position, candidate.tool_quat, frame_id=self._base_frame,
            tolerance_position=tolerance_position,
            tolerance_orientation=tolerance_orientation)
        planned = self.arm.trajectory_final_positions(
            trajectory, self.arm_joint_names) if trajectory is not None else None
        if planned is None:
            candidate.rejection_reason = 'moveit_plan_failed'
            return False
        
        # 3. 用规划预期的终点关节值评估运动学指标
        self._observation_optimizer.evaluate_ik(candidate, planned, planned)
        if candidate.rejection_reason:
            return False
        
        # 4. 执行轨迹
        with self._state_mutex:
            joint_state_sequence = self._joint_state_sequence
        if not self.arm.execute_trajectory(trajectory):
            candidate.rejection_reason = 'moveit_execution_failed'
            return False
            
        # 5. 执行完成后的实际状态复查（闭环安全确认）
        actual = self._wait_for_joint_state(joint_state_sequence)
        if actual is None:
            candidate.rejection_reason = 'joint_state_unavailable_after_motion'
        else:
            self._observation_optimizer.evaluate_ik(candidate, actual, actual)
            if candidate.rejection_reason:
                if prefix_actual_rejection:
                    candidate.rejection_reason = (
                        f'actual_{candidate.rejection_reason}')
        if candidate.rejection_reason:
            return False
        return True

    def _move_to_recentered_pose(self, candidate, current_joints):
        """Apply the normal collision IK, singularity and joint-margin gates."""
        return self._execute_candidate_motion(
            candidate,
            current_joints=current_joints,
            validate_target_ik=True,
            prefix_actual_rejection=True)

    @staticmethod
    def _hint_available(tree_hint):
        if tree_hint is None or not str(tree_hint.header.frame_id).strip():
            return False
        point = tree_hint.point
        return all(math.isfinite(value) for value in (point.x, point.y, point.z))

    # --------- 从 hint 走到观察位 ---------
    def _move_to_observation(self, tree_hint):
        self._observation_failure_reason = ''
        if not self._hint_available(tree_hint):
            self._observation_failure_reason = 'tree_hint_unavailable'
            self.get_logger().error('[ARM] tree_hint is required for observation')
            return False
        try:
            # 将 tree_hint 从目标坐标系（如 map）转换到机械臂基座坐标系 (alicia_base_link)
            transform = self._tf_buffer.lookup_transform(
                self._base_frame, tree_hint.header.frame_id, rclpy.time.Time())
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            tree_in_base = transform_point(
                (tree_hint.point.x, tree_hint.point.y, tree_hint.point.z),
                (translation.x, translation.y, translation.z),
                (rotation.x, rotation.y, rotation.z, rotation.w))
        except (TransformException, ValueError) as error:
            self._observation_failure_reason = f'observation_tf_failed: {error}'
            self.get_logger().error(f'[ARM] cannot build observation pose: {error}')
            return False
        self._tree_in_base = tree_in_base

        if self._observation_mode == 'joint_presets':
            if not self._select_joint_preset_side(tree_in_base):
                return False

        try:
            # 粗对准仍需已加载的 tool0 -> camera 外参；预设扫描本身不使用它求 IK。
            camera_transform = self._tf_buffer.lookup_transform(
                'tool0', self._camera_frame, rclpy.time.Time())
        except TransformException as error:
            self._observation_failure_reason = f'observation_tf_failed: {error}'
            self.get_logger().error(f'[ARM] cannot load camera mount: {error}')
            return False
        camera_translation = camera_transform.transform.translation
        camera_rotation = camera_transform.transform.rotation
        self._camera_mount = (
            (camera_translation.x, camera_translation.y, camera_translation.z),
            (camera_rotation.x, camera_rotation.y,
             camera_rotation.z, camera_rotation.w),
        )
        if self._observation_mode == 'joint_presets':
            if not self._prepare_joint_preset_observation_candidates():
                return False
            return self._move_to_next_observation()
        try:
            nozzle_transform = self._tf_buffer.lookup_transform(
                'tool0', self._observation_config['nozzle_frame'],
                rclpy.time.Time())
        except TransformException as error:
            self._observation_failure_reason = f'observation_tf_failed: {error}'
            self.get_logger().error(f'[ARM] cannot load nozzle mount: {error}')
            return False
        nozzle_translation = nozzle_transform.transform.translation
        nozzle_rotation = nozzle_transform.transform.rotation
        self._nozzle_mount = (
            (nozzle_translation.x, nozzle_translation.y, nozzle_translation.z),
            (nozzle_rotation.x, nozzle_rotation.y,
             nozzle_rotation.z, nozzle_rotation.w),
        )
        if not self._prepare_observation_candidates():
            return False
        return self._move_to_next_observation()

    def _prepare_observation_candidates(self):
        """生成观察网格，结合碰撞 IK 和 URDF 指标保留少量安全候选。"""
        inputs = self._wait_for_observation_inputs()
        if inputs is None:
            self._observation_failure_reason = 'camera_or_joint_state_unavailable'
            return False
        camera, current_joints = inputs
        started = time.monotonic()
        candidates = self._observation_optimizer.generate(
            self._tree_in_base, self._camera_mount, camera)
        visible_count = len(candidates)
        for candidate in candidates:
            if not candidate.visible:
                continue
            if time.monotonic() - started >= self.config.observation_search_timeout:
                candidate.rejection_reason = 'ik_search_timeout'
                continue
            ik = self.arm.compute_ik(
                candidate.tool_position, candidate.tool_quat, current_joints)
            if ik is None:
                candidate.rejection_reason = 'collision_ik_failed'
                continue
            try:
                self._observation_optimizer.evaluate_ik(
                    candidate, dict(zip(ik.name, ik.position)), current_joints)
            except (KeyError, TypeError, ValueError):
                candidate.rejection_reason = 'incomplete_ik_state'
            if not candidate.rejection_reason:
                self._evaluate_nozzle_plane_candidate(candidate)
        self._observation_candidates = self._observation_optimizer.order_for_tree_scan(
            candidates)[:self.config.observation_max_plans]
        ik_count = sum(candidate.ik_joints is not None for candidate in candidates)
        servo_safe_count = sum(
            candidate.visible and not candidate.rejection_reason
            for candidate in candidates)
        self.get_logger().info(
            f'[ARM][OBSERVE] tree_in_base=({self._tree_in_base[0]:.2f},'
            f'{self._tree_in_base[1]:.2f},{self._tree_in_base[2]:.2f}) '
            f'camera={camera[4]}x{camera[5]} fx={camera[0]:.1f} fy={camera[1]:.1f} '
            f'generated={len(candidates)} view_usable={visible_count} '
            f'ik_valid={ik_count} servo_safe={servo_safe_count}')
        self._observation_candidate_index = -1
        if not self._observation_candidates:
            if not candidates:
                reason = 'no_observation_candidate'
            elif visible_count == 0:
                reason = 'no_observation_candidate'
            elif ik_count == 0:
                reason = 'no_collision_free_ik_candidate'
            else:
                reason = 'no_servo_safe_candidate'
            self._observation_failure_reason = reason
            return False
        if self._observation_candidates[0].selection_phase == \
                'center_unavailable_fallback':
            candidate = self._observation_candidates[0]
            self.get_logger().warn(
                '[ARM][OBSERVE] no safe center observation candidate; '
                f'falling back to azimuth={candidate.azimuth_deg:+.0f} deg')
        return True

    def _evaluate_nozzle_plane_candidate(self, candidate):
        """Keep only candidates whose planned nozzle faces the tree plane."""
        try:
            nozzle_position, nozzle_quat = nozzle_pose_from_tool_pose(
                candidate.tool_position, candidate.tool_quat,
                self._nozzle_mount[0], self._nozzle_mount[1])
            distance_m, intersection_m = nozzle_tree_plane_metrics(
                self._tree_in_base, nozzle_position, nozzle_quat)
        except (TypeError, ValueError) as error:
            candidate.rejection_reason = f'nozzle_plane_invalid:{error}'
            return
        minimum = self._observation_config['nozzle_plane_min_m']
        maximum = self._observation_config['nozzle_plane_max_m']
        if not minimum <= distance_m <= maximum:
            candidate.rejection_reason = 'nozzle_plane_distance_out_of_bounds'
            return
        candidate.nozzle_plane_distance_m = distance_m
        candidate.nozzle_axis_plane_intersection_m = intersection_m
        candidate.nozzle_plane_error_m = abs(
            distance_m -
            self._observation_config['preferred_nozzle_plane_distance_m'])

    def _actual_nozzle_plane_metrics(self):
        """Read the executed nozzle pose for diagnostics after MoveIt motion."""
        try:
            transform = self._tf_buffer.lookup_transform(
                self._base_frame, self._observation_config['nozzle_frame'],
                rclpy.time.Time()).transform
            return nozzle_tree_plane_metrics(
                self._tree_in_base,
                (transform.translation.x, transform.translation.y,
                 transform.translation.z),
                (transform.rotation.x, transform.rotation.y,
                 transform.rotation.z, transform.rotation.w)), ''
        except (TransformException, ValueError) as error:
            return None, str(error)

    def _move_to_observation_index(self, index):
        """Move to one known observation view without traversing recovery views."""
        if not 0 <= int(index) < len(self._observation_candidates):
            return False
        self._observation_candidate_index = int(index) - 1
        return self._move_to_next_observation(stop_index=int(index))

    def _move_to_next_observation(self, excluded_indices=None, stop_index=None):
        excluded = set(excluded_indices or ())
        while self._observation_candidate_index + 1 < len(
                self._observation_candidates):
            self._observation_candidate_index += 1
            if (stop_index is not None and
                    self._observation_candidate_index > int(stop_index)):
                return False
            if self._observation_candidate_index in excluded:
                continue
            candidate = self._observation_candidates[self._observation_candidate_index]
            if self._aborted(lambda: False):
                return False
            if getattr(candidate, 'observation_mode', 'ik') == 'joint_presets':
                if not self.arm.move_joints(candidate.joint_positions):
                    candidate.rejection_reason = 'joint_preset_move_failed'
                    self.get_logger().warn(
                        '[ARM][OBSERVE] mode=joint_presets failed '
                        f'index={self._observation_candidate_index} '
                        f'id={candidate.candidate_id}; trying next preset')
                    continue
                camera_pose = self._current_camera_pose()
                if camera_pose is not None:
                    candidate.camera_position, candidate.camera_quat = camera_pose
                    candidate.camera_height_m = candidate.camera_position[2]
                working_range, _error = self._dynamic_nozzle_range()
                self._observation_distance = working_range
                self._observation_pose = None
                joints_deg = ','.join(
                    f'{math.degrees(value):.1f}'
                    for value in candidate.joint_positions)
                self.get_logger().info(
                    '[ARM][OBSERVE] mode=joint_presets selected '
                    f'index={self._observation_candidate_index} '
                    f'id={candidate.candidate_id} joints_deg=[{joints_deg}]')
                return True
            if self._execute_candidate_motion(
                    candidate,
                    tolerance_position=self._observation_config[
                        'position_tolerance_m'],
                    tolerance_orientation=self._observation_config[
                        'orientation_tolerance_rad']):
                self._observation_distance = candidate.distance_m
                self._observation_pose = (candidate.tool_position, candidate.tool_quat)
                actual_nozzle, nozzle_error = (None, 'not sampled')
                actual_metrics = getattr(
                    self, '_actual_nozzle_plane_metrics', None)
                if actual_metrics is not None:
                    actual_nozzle, nozzle_error = actual_metrics()
                nozzle_text = (
                    f' nozzle_plane={actual_nozzle[0]:.2f}m'
                    f' ray={actual_nozzle[1]:.2f}m'
                    if actual_nozzle is not None else
                    f' nozzle_plane=unavailable({nozzle_error})')
                planned_nozzle_distance = getattr(
                    candidate, 'nozzle_plane_distance_m', math.nan)
                planned_nozzle_error = getattr(
                    candidate, 'nozzle_plane_error_m', math.nan)
                self.get_logger().info(
                    f'[ARM][ALIGN] selected observation candidate '
                    f'index={self._observation_candidate_index} '
                    f'id={candidate.candidate_id} distance={candidate.distance_m} m '
                    f'phase={getattr(candidate, "selection_phase", "recovery")} '
                    f'camera_height_in_base={candidate.camera_height_m:.2f} m '
                    f'camera_z_in_base={candidate.camera_position[2]:.2f} m '
                    f'planned_nozzle_plane={planned_nozzle_distance:.2f} m '
                    f'nozzle_error={planned_nozzle_error:.2f} m '
                    f'condition={candidate.condition_number:.2f} '
                    f'joint_margin={candidate.min_joint_margin_rad:.2f}'
                    f'{nozzle_text}')
                return True
            if candidate.rejection_reason == 'moveit_execution_failed':
                self.get_logger().warn(
                    f'[ARM][ALIGN] planning failed for observation candidate '
                    f'index={self._observation_candidate_index} '
                    f'id={candidate.candidate_id}')
        self._observation_failure_reason = 'all_observation_motion_candidates_failed'
        return False

    def _wait_for_state(self, *, require_camera=False, after_sequence=None):
        """等待共享状态快照，统一相机/关节输入的超时与轮询语义。"""
        deadline = time.monotonic() + self.config.observation_input_timeout
        while time.monotonic() < deadline:
            with self._state_mutex:
                camera = self._camera_model
                joints = self._joint_positions
                sequence = self._joint_state_sequence
            if (joints is not None and
                    (not require_camera or camera is not None) and
                    (after_sequence is None or sequence > after_sequence)):
                return (camera, joints) if require_camera else joints
            time.sleep(0.02)
        return None

    def _wait_for_observation_inputs(self):
        return self._wait_for_state(require_camera=True)

    def _current_camera_pose(self):
        """返回机械臂 base 下 C10 optical frame 的最新实际 TF 位姿。

        MoveIt 的 ``SUCCEEDED`` 只说明控制器接受并完成了轨迹容差内的执行；视觉
        重心要求以真实相机轴为基准继续计算，因此不能复用候选生成阶段的理想位姿。
        该函数不等待、不重试：调用方已有任务级恢复路径，TF 短暂不可用时应安全地
        放弃当前观察候选，而不是依据陈旧位姿继续运动。
        """
        try:
            transform = self._tf_buffer.lookup_transform(
                self._base_frame, self._camera_frame, rclpy.time.Time())
        except TransformException as error:
            self.get_logger().debug(
                f'[ARM][RECENTER] actual camera TF unavailable: {error}')
            return None
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        values = (
            translation.x, translation.y, translation.z,
            rotation.x, rotation.y, rotation.z, rotation.w,
        )
        if not all(math.isfinite(float(value)) for value in values):
            self.get_logger().warn(
                '[ARM][RECENTER] actual camera TF contains non-finite values')
            return None
        return (
            (float(translation.x), float(translation.y), float(translation.z)),
            (float(rotation.x), float(rotation.y), float(rotation.z),
             float(rotation.w)),
        )

    def _wait_for_joint_state(self, after_sequence=None):
        return self._wait_for_state(after_sequence=after_sequence)

    def _return_to_observation(self):
        if self._abort.is_set():
            return False
        index = self._observation_candidate_index
        if (0 <= index < len(self._observation_candidates) and
                getattr(self._observation_candidates[index], 'observation_mode', 'ik')
                == 'joint_presets'):
            return self.arm.move_joints(
                self._observation_candidates[index].joint_positions)
        if self._observation_pose is None:
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
