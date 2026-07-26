"""ExecuteSpray 长时流程的失败恢复策略。

节点接线留在 ``spray_task.py``，下游 Action 等待留在 ``action_flow.py``；这里
仅保留“对齐失败后下一观察位/安全 HOME”的状态机分支，避免主节点继续膨胀。
"""

from wvcsc_interfaces.action import AlignTarget, ExecuteSpray

from .target_flow import TargetAttempt


class SpraySequenceMixin:
    def _recover_failure(self, result_code, message, cancel_requested,
                         home_failure_message=None):
        if self._aborted(cancel_requested):
            return ExecuteSpray.Result.CANCELED, 'spray goal canceled'
        if not self._return_home(cancel_requested):
            return ExecuteSpray.Result.HOME_FAILED, (
                home_failure_message or f'{message}; HOME motion failed')
        return result_code, message

    def _alignment_retry_allowed(self, attempt_count):
        return attempt_count < int(self.get_parameter('max_alignment_attempts').value)

    @staticmethod
    def _alignment_code_allows_endpoint_spray(align_code):
        return align_code in {
            AlignTarget.Result.TIMEOUT,
            AlignTarget.Result.TARGET_STALE,
            AlignTarget.Result.SERVO_SINGULARITY,
        }

    @staticmethod
    def _alignment_code_allows_fallback(align_code):
        return align_code in {
            AlignTarget.Result.TIMEOUT,
            AlignTarget.Result.TARGET_STALE,
        }

    @staticmethod
    def _is_recoverable_alignment_code(align_code):
        return align_code in {
            AlignTarget.Result.TIMEOUT,
            AlignTarget.Result.TARGET_STALE,
            AlignTarget.Result.SERVO_SINGULARITY,
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
            attempt = TargetAttempt(target)
            attempts.append(attempt)
        current = self._observation_candidate_index
        if current >= 0:
            attempt.recentered_observation_indices.add(current)
        self._select_target('')
        self._set_inference_mode('idle')
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
            feedback(ExecuteSpray.Feedback.SCANNING_TREE, 0.42, 'SCANNING_TREE')
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
