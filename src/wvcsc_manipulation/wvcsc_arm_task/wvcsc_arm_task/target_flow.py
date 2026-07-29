# 中文说明：机械臂任务的 ROS 视觉目标流程层。
# 目标纯数据、去重、跨视角关联和计数由 target_ledger 提供，本模块负责订阅回调和时序。
"""ROS-facing target flow for one ExecuteSpray goal.

Pure target snapshots, tree-plane geometry, de-duplication and accounting live
in :mod:`target_ledger`.  They remain re-exported here to preserve the
existing Python import path used by integrations and tests.
"""

import math
import time

from wvcsc_interfaces.action import ExecuteSpray

from .target_ledger import (
    FruitTarget,
    TargetAttempt,
    associate_known_targets,
    deduplicate_candidates,
    detection_candidates,
    limit_targets_per_tree,
    spray_summary,
    stable_candidates_by_presence,
    stable_candidates_from_frames,
    target_accounting,
    target_accounting_is_complete,
    target_on_tree_plane,
    target_pixel_error,
    target_requires_recenter,
)


def final_spray_outcome(sprayed, unresolved, saw_disease, summary):
    """Return the existing ExecuteSpray result code for the final ledger state."""
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
    """Return whether the parent Action may report COMPLETED feedback."""
    return result_code in {
        ExecuteSpray.Result.OK,
        ExecuteSpray.Result.PARTIAL_SUCCESS,
        ExecuteSpray.Result.INSPECTED_NO_DISEASE,
    }


class TargetFlowMixin:
    """ROS callbacks and task-local target-flow orchestration for ``SprayTask``."""

    def _on_fruit_detections(self, message):
        """Store one de-duplicated disease-target frame for short-window matching."""
        fruits = deduplicate_candidates(detection_candidates(
            message, self.config.target_class_name,
            self.config.disease_confidence))
        with self._vision_mutex:
            self._fruit_frames += 1
            self._fruit_history.append((time.monotonic(), list(fruits)))

    def _on_selected_target(self, message):
        """Update the selected-target stability window used before visual servo."""
        with self._vision_mutex:
            matching_target = (
                message.target_id == self._target_confirmation_id)
            if not (
                    message.valid and matching_target and
                    message.image_width > 0 and message.image_height > 0):
                self._target_valid_frames = 0
                self._target_workspace_currently_valid = False
                now = time.monotonic()
                short_expected_gap = (
                    not message.valid and
                    (matching_target or not message.target_id) and
                    self._target_workspace_last_seen is not None and
                    now - self._target_workspace_last_seen <=
                    self._recenter_config['post_max_gap_sec'])
                if not short_expected_gap:
                    self._target_confirmation_frames = 0
                    self._target_workspace_stable_since = None
                    self._target_workspace_last_seen = None
                    self._target_workspace_anchor = None
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

            desired_aim = self._active_aim_pixel(
                message.image_width, message.image_height)
            reliable_in_workspace = (
                desired_aim is not None
                and math.isfinite(message.confidence)
                and float(message.confidence) >=
                self._recenter_config['post_min_confidence']
                and not target_requires_recenter(
                    message.center_u, message.center_v, *desired_aim,
                    self._recenter_config['servo_entry_px']))

            if reliable_in_workspace:
                now = time.monotonic()
                point = (float(message.center_u), float(message.center_v))
                if (self._target_workspace_last_seen is not None and
                        now - self._target_workspace_last_seen >
                        self._recenter_config['post_max_gap_sec']):
                    self._target_confirmation_frames = 0
                    self._target_workspace_stable_since = None
                    self._target_workspace_anchor = None

                anchor = self._target_workspace_anchor
                if (anchor is None or math.hypot(
                        point[0] - anchor[0], point[1] - anchor[1]) >
                        self._recenter_config['post_max_drift_px']):
                    self._target_workspace_anchor = point
                    self._target_workspace_stable_since = now
                    self._target_confirmation_frames = 0

                self._target_workspace_last_seen = now
                self._target_workspace_currently_valid = True
                self._target_confirmation_frames += 1
            else:
                self._target_confirmation_frames = 0
                self._target_workspace_stable_since = None
                self._target_workspace_last_seen = None
                self._target_workspace_anchor = None
                self._target_workspace_currently_valid = False

    def _wait_for_fruits(self, cancel_requested):
        """Collect a stable, tracker-ID-independent disease-target frame window."""
        deadline = time.monotonic() + float(self.get_parameter('detection_timeout_sec').value)
        settle = float(
            self.get_parameter('fruit_collection_settle_sec').value)
        required = int(self.get_parameter('confirmation_frames').value)
        collection_started_at = None
        result = None
        while time.monotonic() < deadline:
            if self._aborted(cancel_requested):
                return None
            with self._vision_mutex:
                if collection_started_at is None:
                    collection_started_at = next(
                        (stamp for stamp, candidates in self._fruit_history
                         if candidates), None)
                if (collection_started_at is not None and
                        time.monotonic() - collection_started_at >= settle):
                    frames = [
                        candidates for stamp, candidates in self._fruit_history
                        if collection_started_at <= stamp <=
                        collection_started_at + settle
                    ]
                    result = stable_candidates_from_frames(
                        frames, required,
                        float(self.get_parameter('processed_iou_threshold').value),
                        float(self.get_parameter(
                            'processed_center_distance_px').value))
            if result is not None:
                return self._anchor_targets_to_tree_plane(result)
            time.sleep(0.02)
        with self._vision_mutex:
            result = [] if self._fruit_frames else None
        return (None if result is None else
                self._anchor_targets_to_tree_plane(result))

    def _wait_for_discovery_targets(self, cancel_requested):
        """Freeze stable targets after an explicit static-view presence scan.

        This discovery gate is intentionally separate from the short recheck
        used after the ledger is frozen.  A complete window counts every valid
        YOLO frame, including frames without a box, as its denominator.
        """
        started = time.monotonic()
        duration = self.config.view_detection_duration
        presence_window = self.config.target_presence_window
        presence_ratio = self.config.target_presence_ratio
        minimum_frames = self.config.target_presence_min_frames
        maximum = self.config.max_targets_per_tree
        deadline = started + duration
        latest_stable = []

        while time.monotonic() < deadline:
            if self._aborted(cancel_requested):
                return None
            now = time.monotonic()
            window_start = max(started, now - presence_window)
            with self._vision_mutex:
                frames = [
                    candidates for stamp, candidates in self._fruit_history
                    if window_start <= stamp <= now]
            if now - started >= presence_window:
                latest_stable = stable_candidates_by_presence(
                    frames, presence_ratio, minimum_frames,
                    self.config.processed_iou_threshold,
                    self.config.processed_center_distance_px)
                if maximum > 0 and len(latest_stable) >= maximum:
                    return self._anchor_targets_to_tree_plane(
                        latest_stable[:maximum])
            time.sleep(0.02)

        with self._vision_mutex:
            received_frames = any(
                stamp >= started for stamp, _candidates in self._fruit_history)
        if not received_frames:
            return None
        if maximum > 0:
            latest_stable = latest_stable[:maximum]
        return self._anchor_targets_to_tree_plane(latest_stable)

    def _reset_target_confirmation(self, target_id, *, clear_latest=True):
        """Reset one selected-target confirmation window."""
        with self._vision_mutex:
            self._target_confirmation_id = target_id
            self._target_valid_frames = 0
            self._target_confirmation_frames = 0
            self._target_workspace_stable_since = None
            self._target_workspace_last_seen = None
            self._target_workspace_anchor = None
            self._target_workspace_currently_valid = False
            if clear_latest:
                self._latest_selected_target = None

    def _latest_target(self):
        with self._vision_mutex:
            return self._latest_selected_target

    def _wait_for_target_confirmation(
            self, target_id, cancel_requested, *, require_workspace):
        """Wait for the selected target to meet the existing stability gate."""
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
                    (self._target_workspace_currently_valid and
                     stable_duration >= self._recenter_config['post_stable_sec']))
                if frames >= required and stable_enough:
                    return True
            time.sleep(0.02)
        return False

    def _lock_target(self, target_id, cancel_requested):
        """Lock the latest target snapshot before any physical motion."""
        self._reset_target_confirmation(target_id)
        self._select_target(target_id)
        self._set_inference_mode('target')
        if not self._wait_for_target_confirmation(
                target_id, cancel_requested, require_workspace=False):
            return None
        return self._latest_target()

    def _queue(self, candidates, excluded):
        """Filter resolved targets and sort the remaining targets by aim priority."""
        kept = [
            candidate for candidate in candidates
            if not any(
                self._same_target(candidate, previous)
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

    def _associate_known_targets(self, known, candidates):
        """Link a fresh view to the immutable tree-level target ledger."""
        maximum = float(self.get_parameter(
            'cross_view_reassociation_max_distance_px').value)
        return associate_known_targets(
            known, candidates, self._same_target, maximum)

    def _remember_targets(self, known, candidates):
        """Update known targets with a same-target detection or append a new one."""
        for candidate in candidates:
            for index, previous in enumerate(known):
                if self._same_target(candidate, previous):
                    known[index] = candidate
                    break
            else:
                known.append(candidate)

    def _merge_discovered_targets(self, known, candidates):
        """Merge IK survey views while retaining the clearest observation."""
        for candidate in candidates:
            matching = next((index for index, previous in enumerate(known)
                             if self._same_target(candidate, previous)), None)
            if matching is None:
                known.append(candidate)
            elif candidate.confidence > known[matching].confidence:
                known[matching] = candidate

    def _replace_known_target(self, known, previous, current):
        """Replace a known snapshot after a target reappears in a new view."""
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
        """Return known targets that have not been treated or marked unresolved."""
        resolved = processed + exhausted
        return [
            target for target in known
            if not any(self._same_target(target, previous) for previous in resolved)
        ]

    def _attempt_for(self, candidate, attempts):
        """Return an existing retry record for the same logical target."""
        return next((attempt for attempt in attempts if self._same_target(
            candidate, getattr(attempt, 'ledger_target', None) or attempt.target)), None)

    def _mark_unresolved(self, target, exhausted):
        """Append a target to the unresolved ledger exactly once."""
        if not any(self._same_target(target, previous) for previous in exhausted):
            exhausted.append(target)

    def _same_target(self, candidate, previous):
        """Apply the existing tree-plane, IoU and pixel-distance identity gates."""
        try:
            plane_gate = float(self.get_parameter(
                'cross_view_target_distance_m').value)
        except (AttributeError, KeyError, TypeError, ValueError):
            plane_gate = 0.0
        if (plane_gate > 0.0 and
                candidate.tree_plane_distance_to(previous) <= plane_gate):
            return True
        return (
            candidate.iou(previous) >= float(
                self.get_parameter('processed_iou_threshold').value) or
            candidate.distance_to(previous) <= float(
                self.get_parameter('processed_center_distance_px').value))

    def _anchor_targets_to_tree_plane(self, candidates):
        """Attach a stable spatial proxy after the arm is stationary."""
        try:
            camera_pose = self._current_camera_pose()
        except (AttributeError, TypeError, ValueError):
            camera_pose = None
        with self._state_mutex:
            camera_model = self._camera_model
        return [target_on_tree_plane(
            candidate, camera_pose, camera_model, self._tree_in_base,
            self._observation_candidate_index)
            for candidate in candidates]

    def _reset_vision(self):
        self._observation_pose = None
        self._observation_candidates = []
        self._observation_candidate_index = -1
        self._observation_distance = None
        self._tree_in_base = None
        self._camera_mount = None
        self._reset_fruit_tracking()

    def _reset_fruit_tracking(self):
        with self._vision_mutex:
            self._fruit_frames = 0
            self._fruit_history = []
            self._target_confirmation_id = ''
            self._target_valid_frames = 0
            self._target_confirmation_frames = 0
            self._target_workspace_stable_since = None
            self._target_workspace_last_seen = None
            self._target_workspace_anchor = None
            self._target_workspace_currently_valid = False
            self._latest_selected_target = None
