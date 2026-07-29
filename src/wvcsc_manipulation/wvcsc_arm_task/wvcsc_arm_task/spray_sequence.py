"""ExecuteSpray 状态机及其失败恢复策略。"""

import time

from wvcsc_interfaces.action import AlignTarget, ExecuteSpray

from .spray_workflow import SpraySession
from .target_flow import (
    completion_feedback_allowed,
    final_spray_outcome,
)
from .target_ledger import (
    TargetAttempt,
    limit_targets_per_tree,
    target_accounting_is_complete,
)


class SpraySequenceMixin:
    def _run_sequence(self, request, cancel_requested, feedback):
        """
        执行一棵树的完整闭环，并返回 ``ExecuteSpray`` 结果码和摘要。

        每次喷洒后回到观察位重新检测，避免机械臂运动导致旧图像坐标失效。目标集合
        按几何关系跨轮合并；循环退出前强制满足
        ``detected == sprayed + unresolved``，从而禁止病果静默丢失。
        """
        self.get_logger().info(
            f'[ARM] GOAL_ACCEPTED mission={request.mission_id.strip()} '
            f'spray_duration={request.spray_duration:.1f}s')
        self._reset_vision()
        self._set_inference_mode('idle')

        # 阶段 1: MOVING_TO_OBSERVE（动态计算观察位姿并执行）
        self.get_logger().info(
            f'[ARM] OBSERVE mode={self._observation_mode} '
            'preparing observation motion from tree_hint...')
        feedback(ExecuteSpray.Feedback.MOVING_TO_OBSERVE, 0.05, 'MOVING_TO_OBSERVE')
        if not self._move_to_observation(request.tree_hint):
            failure = self._observation_failure_reason or 'unknown observation failure'
            return self._recover_failure(
                ExecuteSpray.Result.OBSERVE_FAILED,
                f'observation motion failed: {failure}', cancel_requested,
                'observation and HOME motion failed')
        self.get_logger().info(
            f'[ARM] OBSERVE selected distance={self._observation_distance}m '
            f'index={self._observation_candidate_index} '
            f'tree_in_base=({self._tree_in_base[0]:.2f},{self._tree_in_base[1]:.2f},'
            f'{self._tree_in_base[2]:.2f})')

        session = SpraySession()

        def relay_alignment_feedback(message):
            """Keep the parent Action alive while the child Servo is active."""
            now = time.monotonic()
            if now - session.last_alignment_feedback_at < 1.0:
                return
            session.last_alignment_feedback_at = now
            downstream = message.feedback
            feedback(
                ExecuteSpray.Feedback.ALIGNING, 0.45,
                'ALIGNING '
                f'phase={downstream.phase} '
                f'error_px=({downstream.error_u:.1f},{downstream.error_v:.1f})')

        discovery_complete = False
        if self._observation_mode == 'ik':
            feedback(ExecuteSpray.Feedback.DETECTING_TARGETS, 0.20,
                     'DISCOVERING_ALL_VIEWS')
            discovered = self._discover_ik_targets(cancel_requested, feedback)
            if discovered is None:
                return self._recover_failure(
                    ExecuteSpray.Result.VISION_FAILED,
                    'disease detector did not provide frames during IK discovery',
                    cancel_requested)
            session.known_targets = discovered
            session.saw_disease = bool(discovered)
            discovery_complete = True
            self.get_logger().info(
                '[ARM] IK_TARGET_LEDGER stable='
                f'{len(session.known_targets)} '
                f'ids=({",".join(target.target_id for target in session.known_targets)})')

        # 阶段 2/4/5/6/7 循环：检测 - 排队 - 对准 - 喷洒 - 复检
        while True:
            if discovery_complete and not session.known_targets:
                break
            self._set_inference_mode('disease')
            feedback(ExecuteSpray.Feedback.DETECTING_TARGETS, 0.25, 'DETECTING_TARGETS')
            self.get_logger().debug(
                f'[ARM] DETECT inference_mode=disease '
                f'timeout={self.config.detection_timeout:.1f}s '
                f'confirmation={self.config.confirmation_frames}')

            # 等待 YOLO 返回稳定的病态目标检测帧
            frame_candidates = self._wait_for_fruits(cancel_requested)
            if frame_candidates is None:
                return self._recover_failure(
                    ExecuteSpray.Result.VISION_FAILED,
                    'disease detector did not provide frames', cancel_requested)

            if not session.known_targets:
                # 首个非空检测集合决定当前任务点的逻辑目标账本。之后只允许
                # 对该不可新增的集合做空间重关联，避免观察位切换扩张喷洒范围。
                candidates = limit_targets_per_tree(
                    (), frame_candidates,
                    self.config.max_targets_per_tree,
                    self._same_target)
                if not candidates:
                    if self._move_to_next_fan_observation():
                        self._reset_fruit_tracking()
                        continue
                    break
                self._remember_targets(session.known_targets, candidates)
                self.get_logger().info(
                    f'[ARM] TARGET_LEDGER stable='
                    f'{len(session.known_targets)} '
                    f'ids=({",".join(target.target_id for target in session.known_targets)})')
                associations = [
                    (candidate, candidate, False) for candidate in candidates]
            else:
                # A new observation can legitimately produce new perception
                # IDs and a large image displacement.  Associate detections to
                # the frozen tree-level ledger before filtering treated targets;
                # never add a new logical target after the first stable scan.
                associations = self._associate_known_targets(
                    session.known_targets, frame_candidates)
                resolved = session.processed + session.exhausted
                associations = [
                    item for item in associations
                    if not any(self._same_target(item[0], previous)
                               for previous in resolved)
                ]
                candidates = [current for _logical, current, _forced in associations]
                for logical, current, forced in associations:
                    if forced:
                        self.get_logger().info(
                            '[ARM] TARGET_REASSOCIATED '
                            f'logical={logical.target_id} observed={current.target_id} '
                            f'distance={logical.distance_to(current):.1f}px')
            session.saw_disease = bool(session.known_targets)

            # 阶段 4: QUEUING (基于 IoU 和中心距离去重排序)
            feedback(ExecuteSpray.Feedback.QUEUING, 0.35, 'QUEUING')
            # ``candidates`` already excludes targets whose immutable ledger
            # row is TREATED/UNRESOLVED.  Never overwrite that ledger row with
            # a new detector UUID: doing so can make a sprayed leaf pending
            # again after the next observation return.
            queue = self._queue(candidates, ())
            logical_by_current_id = {
                current.target_id: logical
                for logical, current, _forced in associations
            }
            if session.pending_attempt is not None and queue:
                pending_ledger = (
                    session.pending_attempt.ledger_target or
                    session.pending_attempt.target)
                pending_matches = [
                    candidate for candidate in queue
                    if self._same_target(
                        logical_by_current_id.get(candidate.target_id, candidate),
                        pending_ledger)
                ]
                target = min(
                    pending_matches or queue,
                    key=lambda item: item.distance_to(
                        session.pending_attempt.target))

            self.get_logger().info(
                f'[ARM] DETECT_QUEUE candidates={len(candidates)} '
                f'ids=({",".join(c.target_id for c in candidates[:8])})'
                f'{"..." if len(candidates) > 8 else ""} '
                f'processed={len(session.processed)} '
                f'exhausted={len(session.exhausted)} '
                f'queued={len(queue)}')

            # 若当前视野内无病果，进入逻辑检查：
            if not queue:
                pending_targets = self._pending_targets(
                    session.known_targets,
                    session.processed,
                    session.exhausted)
                missing_target = (
                    (session.pending_attempt.ledger_target or
                     session.pending_attempt.target)
                    if session.pending_attempt is not None
                    else (pending_targets[0] if pending_targets else None))
                if missing_target is not None:
                    (attempt, recovered, _moved) = self._recover_missing_target(
                        missing_target,
                        session.pending_attempt,
                        session.attempts,
                        cancel_requested, feedback)
                    if recovered:
                        session.pending_attempt = attempt
                        continue
                if session.pending_attempt is not None:
                    self._mark_unresolved(
                        session.pending_attempt.ledger_target or
                        session.pending_attempt.target, session.exhausted)
                    self.get_logger().warn(
                        f'[ARM][ALIGN] target='
                        f'{session.pending_attempt.target.target_id} '
                        'was not redetected after exhausting safe observation views; '
                        'marked unresolved')
                    session.pending_attempt = None
                for target in self._pending_targets(
                        session.known_targets,
                        session.processed,
                        session.exhausted):
                    self._mark_unresolved(target, session.exhausted)
                    self.get_logger().warn(
                        f'[ARM][QUEUE] target={target.target_id} disappeared '
                        'after exhausting safe observation views; marked unresolved')
                self.get_logger().info(
                    f'[ARM] DETECT queue empty '
                    f'(processed={len(session.processed)} '
                    f'exhausted={len(session.exhausted)}) → breaking loop')
                break

            # 阶段 5: ALIGNING (锁定目标，单次 MoveIt 对准或 IBVS 闭环)
            if session.pending_attempt is not None:
                attempt = session.pending_attempt
                attempt.target = target
                if attempt.ledger_target is None:
                    attempt.ledger_target = logical_by_current_id.get(
                        target.target_id, target)
                session.pending_attempt = None
            else:
                target = queue[0]
                ledger_target = logical_by_current_id.get(target.target_id, target)
                attempt = self._attempt_for(ledger_target, session.attempts)
                if attempt is None:
                    attempt = TargetAttempt(target, ledger_target=ledger_target)
                    session.attempts.append(attempt)
                else:
                    attempt.target = target
                    attempt.ledger_target = ledger_target

            feedback(ExecuteSpray.Feedback.ALIGNING, 0.40, 'LOCKING_TARGET')
            session.recenter_attempts += 1
            locked_target = self._lock_target(target.target_id, cancel_requested)
            if locked_target is None:
                recentered = False
                recenter_message = 'target was not locked before alignment'
            else:
                aim_ready, aim_message = self._request_spray_aim(cancel_requested)
                if not aim_ready:
                    recentered = False
                    recenter_message = aim_message
                else:
                    feedback(
                        ExecuteSpray.Feedback.ALIGNING, 0.42,
                        'RECENTERING_TARGET')
                    recentered, recenter_message = self._recenter_target(
                        locked_target, attempt, cancel_requested)

            fallback_spray = False
            endpoint_spray = False
            if not recentered:
                if self._aborted(cancel_requested):
                    return ExecuteSpray.Result.CANCELED, 'spray goal canceled'
                fallback_target, fallback_message = (
                    self._alignment_fallback_target(
                        locked_target, cancel_requested))
                if fallback_target is not None:
                    target = fallback_target
                    attempt.target = target
                    fallback_spray = True
                    session.alignment_failures += 1
                    self.get_logger().warn(
                        f'[ARM][ALIGN_FALLBACK] target={target.target_id} '
                        f'alignment_failed={recenter_message}; '
                        'spraying_from_safe_pose')
                else:
                    self._select_target('')
                    self._set_inference_mode('idle')
                    self.get_logger().warn(
                        f'[ARM][ALIGN_FALLBACK] target={target.target_id} '
                        f'alignment_failed={recenter_message}; '
                        f'blocked={fallback_message}; '
                        'trying the next observation candidate')
                    self._rewind_for_untried_observation(attempt)
                    recovered, moved = self._recover_to_next_observation(
                        cancel_requested, feedback,
                        attempt.recentered_observation_indices)
                    if recovered:
                        session.pending_attempt = attempt
                        self._reset_fruit_tracking()
                        continue
                    if moved:
                        session.recenter_failures += 1
                        return self._alignment_recovery_failure(
                            f'target preparation failed: {recenter_message}; '
                            'disease target redetection failed after observation recovery',
                            cancel_requested)
                    session.recenter_failures += 1
                    self._mark_unresolved(
                        attempt.ledger_target or attempt.target, session.exhausted)
                    feedback(ExecuteSpray.Feedback.RETURNING_TO_OBSERVE, 0.40,
                             'RETURNING_TO_OBSERVE')
                    if not self._return_to_observation():
                        return self._alignment_recovery_failure(
                            f'target preparation failed: {recenter_message}; '
                            'observation recovery failed', cancel_requested)
                    self._reset_fruit_tracking()
                    continue

            if not fallback_spray:
                attempt.count += 1
                session.alignment_attempts += 1
                feedback(ExecuteSpray.Feedback.ALIGNING, 0.45, 'ALIGNING')
                self.get_logger().info(
                    f'[ARM][ALIGN] ENTER_VISUAL_SERVO target={target.target_id} '
                    f'attempt={attempt.count}/'
                    f'{self.config.max_alignment_attempts} '
                    f'timeout={self._vision_timeout:.1f}s '
                    f'observation_mode={self._observation_mode}')
                ok, canceled, align_code, message = self._align_target(
                    request.mission_id, target.target_id,
                    self._active_aim, cancel_requested,
                    feedback_callback=relay_alignment_feedback)

                self.get_logger().debug(
                    f'[ARM][ALIGN] result target={target.target_id} '
                    f'code={align_code} message={message}')

            if not fallback_spray and not ok:
                session.alignment_failures += 1
                if canceled:
                    return ExecuteSpray.Result.CANCELED, message
                if align_code == AlignTarget.Result.SERVO_SAFETY_STOP:
                    self._set_inference_mode('idle')
                    self._request_motion_stop()
                    return (
                        ExecuteSpray.Result.LOCKED,
                        f'visual alignment hard safety stop: {message}')
                if (self._spray_on_alignment_failure and
                        self._alignment_code_allows_endpoint_spray(align_code)):
                    endpoint_spray = True
                    self.get_logger().warn(
                        f'[ARM][ALIGN_FALLBACK] target={target.target_id} '
                        f'alignment_failed={message}; '
                        'spraying_from_current_servo_pose')
                else:
                    fallback_target, fallback_message = (
                        self._alignment_fallback_target(
                            locked_target, cancel_requested)
                        if self._alignment_code_allows_fallback(align_code)
                        else (None, f'visual alignment code={align_code}'))
                    if fallback_target is not None:
                        target = fallback_target
                        attempt.target = target
                        fallback_spray = True
                        self.get_logger().warn(
                            f'[ARM][ALIGN_FALLBACK] target={target.target_id} '
                            f'alignment_failed={message}; '
                            'spraying_from_safe_pose')
                    else:
                        self._select_target('')
                        self._set_inference_mode('idle')
                        self.get_logger().warn(
                            f'[ARM][ALIGN_FALLBACK] target={target.target_id} '
                            f'blocked={fallback_message}')
                if endpoint_spray:
                    fallback_spray = True
                if not fallback_spray:
                    recoverable = self._is_recoverable_alignment_code(align_code)
                    if not recoverable:
                        return self._alignment_recovery_failure(
                            f'visual alignment code={align_code}: {message}',
                            cancel_requested)
                    if self._alignment_retry_allowed(attempt.count):
                        self.get_logger().warn(
                            f'[ARM][ALIGN] recoverable failure code={align_code}; '
                            'trying the next observation candidate')
                        self._rewind_for_untried_observation(attempt)
                        recovered, moved = self._recover_to_next_observation(
                            cancel_requested, feedback,
                            attempt.recentered_observation_indices)
                        if recovered:
                            session.pending_attempt = attempt
                            self._reset_fruit_tracking()
                            self.get_logger().info(
                                f'[ARM][ALIGN] recovery ready at '
                                f'{self._observation_distance} m; redetecting fruit')
                            continue
                        if moved:
                            session.recenter_failures += 1
                            return self._alignment_recovery_failure(
                                f'visual alignment code={align_code}: {message}; '
                                'disease target redetection failed after observation recovery',
                                cancel_requested)
                    session.recenter_failures += 1
                    self._mark_unresolved(
                        attempt.ledger_target or attempt.target, session.exhausted)
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

            # 阶段 6: SPRAYING (调用下游喷洒 Action)
            ledger_target = attempt.ledger_target or target
            if (session.sprayed >= len(session.known_targets) or
                    any(self._same_target(ledger_target, previous)
                        for previous in session.processed)):
                message = 'strict single-spray guard blocked duplicate ledger target'
                self.get_logger().error(f'[ARM] {message}')
                return self._recover_failure(
                    ExecuteSpray.Result.VISION_FAILED, message, cancel_requested)
            self._set_inference_mode('idle')
            feedback(ExecuteSpray.Feedback.SPRAYING, 0.60, 'SPRAYING')
            self.get_logger().info(
                f'[ARM] SPRAY target={target.target_id} '
                f'duration={request.spray_duration:.1f}s')
            ok, canceled, message = self._spray_target(
                request.mission_id, request.spray_duration,
                cancel_requested)
            if not ok:
                if canceled:
                    return ExecuteSpray.Result.CANCELED, message
                return self._recover_failure(
                    ExecuteSpray.Result.SPRAY_FAILED, message, cancel_requested)
            self.get_logger().info(
                f'[ARM] SPRAY target={target.target_id} done → TREATED '
                f'({session.sprayed + 1} sprayed so far)')
            session.sprayed += 1
            session.processed.append(ledger_target)
            self._select_target('')

            if endpoint_spray:
                self.get_logger().info(
                    '[ARM] endpoint fallback sprayed; '
                    'returning to observation for remaining targets')
                # Fall through to RETURNING_TO_OBSERVE below
                # instead of breaking the while loop, so remaining
                # queued targets are still processed.

            # 阶段 7: RETURNING_TO_OBSERVE (回到观察位，准备复检)
            feedback(ExecuteSpray.Feedback.RETURNING_TO_OBSERVE, 0.75,
                     'RETURNING_TO_OBSERVE')
            self.get_logger().info(
                f'[ARM] RETURN_TO_OBSERVE distance={self._observation_distance}m')
            if not self._return_to_observation():
                return ExecuteSpray.Result.HOME_FAILED, 'observation return failed'
            self._reset_fruit_tracking()

        # 队列处理后，按快照的传递关联统计物理目标，避免恢复位的框漂移重复计数。
        detected, accounted_sprayed, unresolved, pending = (
            session.accounting(self._same_target))
        for target in pending:
            self._mark_unresolved(target, session.exhausted)
        detected, accounted_sprayed, unresolved, pending = (
            session.accounting(self._same_target))
        if len(session.known_targets) != detected:
            self.get_logger().info(
                f'[ARM] target accounting reconciled '
                f'raw_detected={len(session.known_targets)} '
                f'logical_detected={detected}')
        if (session.sprayed != len(session.processed) or
                session.sprayed != accounted_sprayed or pending or
                not target_accounting_is_complete(
                    detected, accounted_sprayed, unresolved)):
            message = (
                'target accounting invariant failed: '
                f'detected={detected} sprayed={accounted_sprayed} '
                f'unresolved={unresolved} '
                f'treated={len(session.processed)}')
            self.get_logger().error(f'[ARM] {message}')
            return self._recover_failure(
                ExecuteSpray.Result.VISION_FAILED, message, cancel_requested)

        if not self._wait_post_spray_home_delay(
                accounted_sprayed, cancel_requested, feedback):
            return ExecuteSpray.Result.CANCELED, 'spray goal canceled'

        # 阶段尾: RETURNING_HOME
        feedback(ExecuteSpray.Feedback.RETURNING_HOME, 0.90, 'RETURNING_HOME')
        self.get_logger().info('[ARM] HOME returning to home_pose...')
        if not self._return_home(cancel_requested):
            return (ExecuteSpray.Result.CANCELED, 'spray goal canceled') if self._aborted(
                cancel_requested) else (ExecuteSpray.Result.HOME_FAILED, 'HOME motion failed')
        self.get_logger().info('[ARM] HOME reached')

        # 生成任务摘要与结果
        summary = session.result_summary(
            detected, accounted_sprayed, unresolved)
        self.get_logger().info(
            '[ARM] ═══ SUMMARY ═══ '
            f'distance={self._observation_distance}m '
            f'{summary}')
        code, message = final_spray_outcome(
            accounted_sprayed, unresolved, session.saw_disease, summary)
        if completion_feedback_allowed(code):
            feedback(ExecuteSpray.Feedback.COMPLETED, 1.0, 'COMPLETED')
        return code, message

    def _recover_failure(self, result_code, message, cancel_requested,
                         home_failure_message=None):
        if self._aborted(cancel_requested):
            return ExecuteSpray.Result.CANCELED, 'spray goal canceled'
        if not self._return_home(cancel_requested):
            return ExecuteSpray.Result.HOME_FAILED, (
                home_failure_message or f'{message}; HOME motion failed')
        return result_code, message

    def _wait_post_spray_home_delay(
            self, sprayed, cancel_requested, feedback):
        """Keep the completed tree at observation pose before the final HOME."""
        delay = float(getattr(self.config, 'post_spray_home_delay', 0.0))
        if sprayed <= 0 or delay <= 0.0:
            return not self._aborted(cancel_requested)
        feedback(ExecuteSpray.Feedback.RETURNING_HOME, 0.88, 'POST_SPRAY_WAIT')
        self.get_logger().info(
            f'[ARM] POST_SPRAY_WAIT delay={delay:.1f}s')
        deadline = time.monotonic() + delay
        while time.monotonic() < deadline:
            if self._aborted(cancel_requested):
                return False
            time.sleep(min(0.02, max(0.0, deadline - time.monotonic())))
        return not self._aborted(cancel_requested)

    def _alignment_retry_allowed(self, attempt_count):
        return attempt_count < int(self.get_parameter('max_alignment_attempts').value)

    @staticmethod
    def _alignment_code_allows_endpoint_spray(align_code):
        return align_code in {
            AlignTarget.Result.TIMEOUT,
            AlignTarget.Result.TARGET_STALE,
            AlignTarget.Result.SERVO_ACTUATION_STALL,
        }

    @staticmethod
    def _alignment_code_allows_fallback(align_code):
        return align_code in {
            AlignTarget.Result.TIMEOUT,
            AlignTarget.Result.TARGET_STALE,
            AlignTarget.Result.SERVO_DIRECTION_DIVERGENCE,
        }

    @staticmethod
    def _is_recoverable_alignment_code(align_code):
        return align_code in {
            AlignTarget.Result.TIMEOUT,
            AlignTarget.Result.TARGET_STALE,
            AlignTarget.Result.SERVO_SINGULARITY,
            AlignTarget.Result.SERVO_ACTUATION_STALL,
            AlignTarget.Result.SERVO_DIRECTION_DIVERGENCE,
        }

    def _alignment_fallback_target(self, target, cancel_requested):
        if not self._spray_on_alignment_failure:
            return None, 'alignment fallback is disabled'
        if target is None:
            return None, 'target was not confirmed before alignment failure'
        if self._aborted(cancel_requested) or self.state.locked:
            return None, 'motion is canceled or locked'
        if not self._return_to_observation():
            return None, 'could not return to the safe observation pose'
        with self._state_mutex:
            sequence = self._joint_state_sequence
        current_joints = self._wait_for_joint_state(after_sequence=sequence)
        if current_joints is None:
            return None, 'fresh joint state is unavailable after observation return'
        if self._aborted(cancel_requested) or self.state.locked:
            return None, 'motion became canceled or locked'
        preflight_ok, preflight_message = self._motion_preflight(
            target, current_joints, source='alignment_failure_fallback',
            error_norm_px=0.0, stage='FALLBACK')
        if not preflight_ok:
            return None, preflight_message
        self._reset_target_confirmation(target.target_id)
        self._select_target(target.target_id)
        self._set_inference_mode('target')
        if not self._wait_for_target_confirmation(
                target.target_id, cancel_requested, require_workspace=False):
            return None, 'target was not reconfirmed at the safe observation pose'
        confirmed = self._latest_target()
        if confirmed is None:
            return None, 'target snapshot is unavailable after reconfirmation'
        return confirmed, ''

    def _rewind_for_untried_observation(self, attempt):
        current = self._observation_candidate_index
        if current + 1 < len(self._observation_candidates):
            return
        if any(index not in attempt.recentered_observation_indices
               for index in range(max(0, current))):
            self._observation_candidate_index = -1

    def _recover_missing_target(self, target, pending_attempt, attempts,
                                cancel_requested, feedback):
        attempt = pending_attempt or self._attempt_for(target, attempts)
        if attempt is None:
            attempt = TargetAttempt(target, ledger_target=target)
            attempts.append(attempt)
        current = self._observation_candidate_index
        if current >= 0:
            attempt.recentered_observation_indices.add(current)
        self._select_target('')
        self._set_inference_mode('idle')
        if (target.observation_index >= 0 and
                target.observation_index != current and
                self._move_to_observation_index(target.observation_index)):
            self._reset_fruit_tracking()
            self.get_logger().info(
                f'[ARM][QUEUE] target={target.target_id} returning to '
                f'discovery observation index={target.observation_index}')
            return attempt, True, True
        self._rewind_for_untried_observation(attempt)
        recovered, moved = self._recover_to_next_observation(
            cancel_requested, feedback, attempt.recentered_observation_indices)
        if recovered:
            self._reset_fruit_tracking()
            self.get_logger().info(
                f'[ARM][QUEUE] target={target.target_id} missing in current view; '
                f'retrying detection at observation '
                f'index={self._observation_candidate_index}')
        return attempt, recovered, moved

    def _discover_ik_targets(self, cancel_requested, feedback):
        """Survey center and fan views before IK starts any physical spraying."""
        discovered = []
        while True:
            self._set_inference_mode('disease')
            candidates = self._wait_for_fruits(cancel_requested)
            if candidates is None:
                return None
            self._merge_discovered_targets(discovered, candidates)
            self.get_logger().info(
                f'[ARM] IK_DISCOVERY index={self._observation_candidate_index} '
                f'found={len(candidates)} unique={len(discovered)}')
            if not self._move_to_next_fan_observation():
                break
            self._reset_fruit_tracking()
            feedback(ExecuteSpray.Feedback.DETECTING_TARGETS, 0.22,
                     'DISCOVERING_NEXT_VIEW')
        maximum = self.config.max_targets_per_tree
        ranked = sorted(discovered, key=lambda target: target.confidence,
                        reverse=True)
        return ranked if maximum <= 0 else ranked[:maximum]

    def _recover_to_next_observation(self, cancel_requested, feedback,
                                     excluded_indices=None):
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
            feedback(
                ExecuteSpray.Feedback.DETECTING_TARGETS, 0.42,
                'REDETECTING_TARGETS')
            return True, moved
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
