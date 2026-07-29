import json
import math
import threading
import time
from types import SimpleNamespace

from action_msgs.msg import GoalStatus
from wvcsc_interfaces.action import AlignTarget, ExecuteSpray

from wvcsc_arm_task.spray_task import SprayTask
from wvcsc_arm_task.spray_config import target_recenter_parameters
from wvcsc_arm_task.target_flow import (
    FruitTarget, TargetAttempt, completion_feedback_allowed,
    associate_known_targets,
    deduplicate_candidates, detection_candidates, final_spray_outcome,
    limit_targets_per_tree, spray_summary, target_accounting,
    stable_candidates_by_presence, stable_candidates_from_frames,
    target_accounting_is_complete,
    target_requires_recenter)
from wvcsc_arm_task.observation.candidate import ObservationCandidate


def _target(target_id, center_u, center_v, confidence=0.9):
    return FruitTarget(target_id, confidence, center_u, center_v, 40.0, 40.0)


class _QueueHarness:
    _queue = SprayTask._queue
    _remember_targets = SprayTask._remember_targets
    _replace_known_target = SprayTask._replace_known_target
    _pending_targets = SprayTask._pending_targets
    _attempt_for = SprayTask._attempt_for
    _rewind_for_untried_observation = SprayTask._rewind_for_untried_observation
    _same_target = SprayTask._same_target
    _mark_unresolved = SprayTask._mark_unresolved

    def get_parameter(self, name):
        return SimpleNamespace(value={
            'processed_iou_threshold': 0.30,
            'processed_center_distance_px': 18.0,
            'image_width': 1280,
            'image_height': 720,
        }[name])


class _IkDiscoveryHarness:
    _discover_ik_targets = SprayTask._discover_ik_targets
    _merge_discovered_targets = SprayTask._merge_discovered_targets
    _same_target = SprayTask._same_target

    def __init__(self, views):
        self._views = list(views)
        self._view_index = 0
        self._observation_candidate_index = 0
        self.config = SimpleNamespace(max_targets_per_tree=2)

    def get_parameter(self, name):
        return SimpleNamespace(value={
            'processed_iou_threshold': 0.30,
            'processed_center_distance_px': 18.0,
            'cross_view_target_distance_m': 0.08,
        }[name])

    @staticmethod
    def get_logger():
        return SimpleNamespace(info=lambda *_args: None)

    @staticmethod
    def _set_inference_mode(_mode):
        pass

    def _wait_for_fruits(self, _cancel_requested):
        return self._views[self._view_index]

    def _wait_for_discovery_targets(self, cancel_requested):
        return self._wait_for_fruits(cancel_requested)

    def _move_to_next_fan_observation(self):
        if self._view_index + 1 >= len(self._views):
            return False
        self._view_index += 1
        self._observation_candidate_index += 1
        return True

    @staticmethod
    def _reset_fruit_tracking():
        pass


def test_queue_excludes_processed_target_by_geometry_not_only_tracker_id():
    task = _QueueHarness()
    processed = _target('old-id', 640.0, 360.0)
    same_fruit_new_id = _target('new-id', 650.0, 360.0)
    other = _target('other', 800.0, 360.0)
    queue = task._queue([same_fruit_new_id, other], [processed])
    assert queue == [other]


def test_ik_discovery_unions_center_and_fan_views_before_any_spray():
    left = FruitTarget('left-a', 0.8, 200.0, 240.0, 40.0, 40.0,
                       0.00, 1.10, 1)
    right_same_a = FruitTarget('right-a', 0.9, 500.0, 240.0, 40.0, 40.0,
                                0.02, 1.10, 2)
    right_b = FruitTarget('right-b', 0.85, 580.0, 250.0, 40.0, 40.0,
                          0.25, 1.15, 2)
    task = _IkDiscoveryHarness([[], [left], [right_same_a, right_b]])

    discovered = task._discover_ik_targets(lambda: False, lambda *_args: None)

    assert [target.target_id for target in discovered] == ['right-a', 'right-b']


def test_queue_does_not_merge_a_nearby_second_physical_fruit():
    task = _QueueHarness()
    processed = _target('fruit-1', 640.0, 360.0)
    second_fruit = _target('fruit-2', 665.0, 360.0)

    assert task._queue([second_fruit], [processed]) == [second_fruit]


def test_queue_prefers_the_fruit_nearest_the_image_center():
    task = _QueueHarness()
    near = _target('near', 650.0, 365.0, confidence=0.80)
    far = _target('far', 900.0, 600.0, confidence=0.99)
    assert task._queue([far, near], []) == [near, far]


def test_queue_keeps_only_the_highest_confidence_duplicate():
    task = _QueueHarness()
    weaker = _target('fruit-1', 640.0, 360.0, confidence=0.30)
    stronger = _target('fruit-2', 646.0, 364.0, confidence=0.80)

    assert deduplicate_candidates([weaker, stronger]) == [stronger]
    assert task._queue([weaker, stronger], []) == [stronger]


def test_real_target_limit_keeps_the_initial_two_highest_confidence_targets():
    task = _QueueHarness()
    low = _target('low', 200.0, 200.0, confidence=0.30)
    medium = _target('medium', 400.0, 200.0, confidence=0.70)
    high = _target('high', 600.0, 200.0, confidence=0.90)

    selected = limit_targets_per_tree(
        [], [low, medium, high], 2, task._same_target)

    assert [target.target_id for target in selected] == ['high', 'medium']
    assert limit_targets_per_tree(
        selected, [low], 2, task._same_target) == []


def test_target_ledger_merges_reidentified_fruit_by_geometry():
    task = _QueueHarness()
    known = []
    task._remember_targets(known, [
        _target('fruit-1', 640.0, 360.0),
        _target('fruit-2', 800.0, 360.0),
    ])
    task._remember_targets(known, [
        _target('fruit-9', 650.0, 360.0, confidence=0.95),
    ])

    assert len(known) == 2
    assert known[0].target_id == 'fruit-9'


def test_recovery_reassociation_replaces_the_previous_ledger_target():
    task = _QueueHarness()
    previous = _target('fruit-1', 200.0, 200.0)
    known = [previous, _target('fruit-2', 900.0, 500.0)]
    reassociated = _target('fruit-10', 700.0, 360.0)

    task._replace_known_target(known, previous, reassociated)

    assert [target.target_id for target in known] == ['fruit-10', 'fruit-2']


def test_disappeared_pending_target_is_not_silently_completed():
    task = _QueueHarness()
    treated = _target('fruit-1', 640.0, 360.0)
    missing = _target('fruit-2', 800.0, 360.0)
    known = [treated, missing]
    processed = [treated]
    exhausted = []

    for target in task._pending_targets(known, processed, exhausted):
        task._mark_unresolved(target, exhausted)

    assert [target.target_id for target in exhausted] == ['fruit-2']
    assert len(known) == len(processed) + len(exhausted)
    code, _message = final_spray_outcome(
        len(processed), len(exhausted), True,
        spray_summary(len(known), len(processed), len(exhausted), 0, 1, 0, 1))
    assert code == ExecuteSpray.Result.PARTIAL_SUCCESS


def test_target_accounting_requires_every_detection_to_be_resolved():
    assert target_accounting_is_complete(2, 1, 1)
    assert not target_accounting_is_complete(2, 1, 0)


def test_presence_stability_counts_empty_inference_frames():
    target = _target('target-a', 640.0, 360.0, confidence=0.80)
    frames = [[target], [], [target], [], [target], [], [target], [], [target], []]

    stable = stable_candidates_by_presence(frames, 0.50, 5, 0.30, 18.0)

    assert stable == [target]


def test_presence_stability_rejects_insufficient_frames_and_low_ratio():
    target = _target('target-a', 640.0, 360.0, confidence=0.80)

    assert stable_candidates_by_presence(
        [[target]] * 4, 0.50, 5, 0.30, 18.0) == []
    assert stable_candidates_by_presence(
        [[target], [], [], [], []], 0.50, 5, 0.30, 18.0) == []


def test_target_accounting_merges_transitive_recovery_snapshots():
    task = _QueueHarness()
    first = _target('fruit-1', 640.0, 360.0)
    bridge = _target('fruit-9', 655.0, 360.0)
    latest = _target('fruit-18', 670.0, 360.0)
    other = _target('fruit-2', 900.0, 360.0)

    detected, sprayed, unresolved, pending = target_accounting(
        [first, bridge, latest, other], [first], [other], task._same_target)

    assert (detected, sprayed, unresolved, pending) == (2, 1, 1, [])


def test_iou_is_zero_for_disjoint_targets():
    assert _target('a', 100.0, 100.0).iou(_target('b', 300.0, 300.0)) == 0.0


def test_diseased_target_below_point_one_never_enters_the_queue():
    def detection(score):
        return SimpleNamespace(
            id='fruit',
            results=[SimpleNamespace(hypothesis=SimpleNamespace(
                class_id='diseased_target', score=score))],
            bbox=SimpleNamespace(
                center=SimpleNamespace(position=SimpleNamespace(x=640.0, y=360.0)),
                size_x=40.0, size_y=40.0),
        )

    assert detection_candidates(
        SimpleNamespace(detections=[detection(0.099)]), 'diseased_target', 0.10) == []
    assert len(detection_candidates(
        SimpleNamespace(detections=[detection(0.10)]), 'diseased_target', 0.10)) == 1


class _FruitCollectionHarness:
    _wait_for_fruits = SprayTask._wait_for_fruits

    @staticmethod
    def _anchor_targets_to_tree_plane(candidates):
        return candidates

    def __init__(self):
        first = _target('fruit-1', 640.0, 360.0)
        now = time.monotonic()
        self._vision_mutex = threading.Lock()
        self._fruit_frames = 3
        self._fruit_counts = {'fruit-1': 3}
        self._fruit_latest = {'fruit-1': first}
        self._fruit_history = [
            (now, [first]),
            (now + 0.001, [first]),
            (now + 0.002, [first]),
        ]

    @staticmethod
    def _aborted(_cancel_requested):
        return False

    @staticmethod
    def get_parameter(name):
        return SimpleNamespace(value={
            'detection_timeout_sec': 0.20,
            'fruit_collection_settle_sec': 0.06,
            'confirmation_frames': 3,
            'processed_iou_threshold': 0.30,
            'processed_center_distance_px': 18.0,
        }[name])


def test_fruit_collection_waits_for_a_later_stable_target():
    task = _FruitCollectionHarness()

    def add_second_target():
        time.sleep(0.02)
        second = _target('fruit-2', 800.0, 220.0)
        with task._vision_mutex:
            task._fruit_counts['fruit-2'] = 3
            task._fruit_latest['fruit-2'] = second
            now = time.monotonic()
            task._fruit_history.extend([
                (now, [second]),
                (now + 0.001, [second]),
                (now + 0.002, [second]),
            ])

    worker = threading.Thread(target=add_second_target)
    worker.start()
    candidates = task._wait_for_fruits(lambda: False)
    worker.join()

    assert {candidate.target_id for candidate in candidates} == {
        'fruit-1', 'fruit-2'}


def test_stable_collection_keeps_two_fruits_when_tracker_ids_change():
    frames = [
        [_target('fruit-1', 200.0, 220.0),
         _target('fruit-2', 260.0, 220.0)],
        [_target('fruit-5', 202.0, 220.0),
         _target('fruit-6', 258.0, 220.0)],
        [_target('fruit-9', 201.0, 221.0),
         _target('fruit-10', 259.0, 221.0)],
    ]

    stable = stable_candidates_from_frames(frames, 3, 0.30, 18.0)

    assert [candidate.target_id for candidate in stable] == [
        'fruit-9', 'fruit-10']


def test_retry_matches_the_same_fruit_after_tracker_id_changes():
    task = _QueueHarness()
    first = _target('fruit-1', 640.0, 360.0)
    retry = _target('fruit-9', 650.0, 360.0)
    attempt = type('Attempt', (), {'target': first})()
    assert task._attempt_for(retry, [attempt]) is attempt


def test_lost_target_after_recovery_is_recorded_once_as_unresolved():
    task = _QueueHarness()
    exhausted = []
    task._mark_unresolved(_target('fruit-1', 640.0, 360.0), exhausted)
    task._mark_unresolved(_target('fruit-9', 650.0, 360.0), exhausted)

    assert [target.target_id for target in exhausted] == ['fruit-1']
    assert spray_summary(1, 0, len(exhausted), 1, 2, 1, 0) == (
        'detected=1 sprayed=0 unresolved=1 alignment_failures=1 '
        'recenter_attempts=2 recenter_failures=1 alignment_attempts=0')


def test_vision_failure_never_publishes_completed_feedback():
    code, _message = final_spray_outcome(
        sprayed=0, unresolved=1, saw_disease=True, summary='failed')
    assert code != 0
    assert not completion_feedback_allowed(code)
    assert completion_feedback_allowed(
        final_spray_outcome(1, 0, True, 'ok')[0])


class _RecenterParameterHarness:
    _target_recenter_parameters = target_recenter_parameters

    @staticmethod
    def get_parameter(name):
        return SimpleNamespace(value={
            'target_recenter_trigger_px': 48.0,
            'visual_servo_entry_max_error_px': 48.0,
            'target_recenter_max_angle_deg': 45.0,
            'target_recenter_max_total_angle_deg': 45.0,
            'target_recenter_refine_goal_px': 8.0,
            'target_recenter_max_iterations': 1,
            'target_recenter_residual_candidates_px': [0.0],
            'target_recenter_position_tolerance_m': 0.002,
            'target_recenter_orientation_tolerance_rad': 0.002,
            'target_post_recenter_stable_sec': 0.50,
            'target_post_recenter_max_drift_px': 4.0,
            'target_post_recenter_max_gap_sec': 0.20,
            'target_post_recenter_min_confidence': 0.30,
        }[name])


def test_closed_loop_requires_coarse_recenter_before_servo_handoff():
    config = _RecenterParameterHarness()._target_recenter_parameters()

    assert config['trigger_px'] == 48.0
    assert config['servo_entry_px'] == 48.0


def test_closed_loop_rejects_a_servo_window_wider_than_recenter_trigger():
    class InvalidHarness(_RecenterParameterHarness):
        @staticmethod
        def get_parameter(name):
            value = _RecenterParameterHarness.get_parameter(name).value
            if name == 'visual_servo_entry_max_error_px':
                value = 49.0
            return SimpleNamespace(value=value)

    try:
        InvalidHarness()._target_recenter_parameters()
    except ValueError as error:
        assert str(error) == 'target recenter parameters are invalid'
    else:
        raise AssertionError('closed-loop configuration unexpectedly passed')


class _ClosedLoopSequenceHarness:
    _run_sequence = SprayTask._run_sequence
    _queue = SprayTask._queue
    _remember_targets = SprayTask._remember_targets
    _replace_known_target = SprayTask._replace_known_target
    _pending_targets = SprayTask._pending_targets
    _attempt_for = SprayTask._attempt_for
    _same_target = SprayTask._same_target
    _mark_unresolved = SprayTask._mark_unresolved
    _wait_post_spray_home_delay = SprayTask._wait_post_spray_home_delay

    def _associate_known_targets(self, known, candidates):
        return associate_known_targets(
            known, candidates, self._same_target, 320.0)

    def __init__(self, *, alignment_ok=True, fallback_enabled=True,
                 recenter_ok=True, alignment_code=None, targets=None,
                 failed_target_ids=()):
        self._spray_on_alignment_failure = fallback_enabled
        self._observation_mode = 'joint_presets'
        self._observation_candidate_index = 0
        self._observation_distance = 1.0
        self._vision_timeout = 30.0
        self.config = SimpleNamespace(
            detection_timeout=2.0,
            confirmation_frames=3,
            max_alignment_attempts=2,
            max_targets_per_tree=0,
            post_spray_home_delay=0.0,
        )
        self._active_aim = (640.0, 388.0, 1280, 720, 1.0)
        self._tree_in_base = (0.0, 1.6, 0.0)
        self._observed_targets = targets or [_target('target-1', 200.0, 200.0)]
        self._fruit_calls = 0
        self._alignment_ok = alignment_ok
        self._alignment_code = alignment_code
        self._failed_target_ids = set(failed_target_ids)
        self._recenter_ok = recenter_ok
        self.calls = []

    def get_parameter(self, name):
        return SimpleNamespace(value={
            'detection_timeout_sec': 2.0,
            'confirmation_frames': 3,
            'max_alignment_attempts': 2,
            'max_targets_per_tree': 0,
            'processed_iou_threshold': 0.30,
            'processed_center_distance_px': 40.0,
            'cross_view_reassociation_max_distance_px': 320.0,
            'image_width': 1280,
            'image_height': 720,
        }[name])

    @staticmethod
    def get_logger():
        return SimpleNamespace(
            debug=lambda *_args: None,
            info=lambda *_args: None,
            warn=lambda *_args: None,
            error=lambda *_args: None)

    @staticmethod
    def _aborted(_cancel_requested):
        return False

    def _reset_vision(self):
        self.calls.append('reset_vision')

    def _set_inference_mode(self, mode):
        self.calls.append(f'mode:{mode}')

    def _move_to_observation(self, _hint):
        return True

    def _move_to_next_fan_observation(self):
        return False

    def _wait_for_fruits(self, _cancel_requested):
        self._fruit_calls += 1
        return self._observed_targets if self._fruit_calls <= 2 else []

    def _wait_for_discovery_targets(self, cancel_requested):
        return self._wait_for_fruits(cancel_requested)

    def _request_spray_aim(self, _cancel_requested):
        self.calls.append('request_aim')
        return True, ''

    def _lock_target(self, target_id, _cancel_requested):
        self.calls.append(f'lock:{target_id}')
        return next(target for target in self._observed_targets
                    if target.target_id == target_id)

    def _recenter_target(self, target, _attempt, _cancel_requested):
        self.calls.append(f'recenter:{target.target_id}')
        return (
            self._recenter_ok,
            '' if self._recenter_ok else 'target recenter rejected: joint_limit_margin')

    def _align_target(
            self, _mission, target_id, _aim, _cancel_requested,
            feedback_callback=None):
        self.calls.append(f'servo:{target_id}')
        if feedback_callback is not None:
            feedback_callback(SimpleNamespace(feedback=SimpleNamespace(
                phase=0, error_u=2.0, error_v=-3.0)))
        ok = self._alignment_ok and target_id not in self._failed_target_ids
        return (
            ok,
            False,
            (AlignTarget.Result.OK if ok else
             (self._alignment_code or AlignTarget.Result.TIMEOUT)),
            'alignment timeout' if not ok else '')

    def _alignment_fallback_target(self, target, _cancel_requested):
        self.calls.append(f'fallback:{target.target_id}')
        if not self._spray_on_alignment_failure:
            return None, 'alignment fallback is disabled'
        return target, ''

    _alignment_code_allows_fallback = staticmethod(
        SprayTask._alignment_code_allows_fallback)
    _alignment_code_allows_endpoint_spray = staticmethod(
        SprayTask._alignment_code_allows_endpoint_spray)
    _is_recoverable_alignment_code = staticmethod(
        SprayTask._is_recoverable_alignment_code)

    @staticmethod
    def _alignment_retry_allowed(_count):
        return False

    def _spray_target(self, _mission, _duration, _cancel_requested):
        self.calls.append('spray')
        return True, False, ''

    def _alignment_recovery_failure(self, message, _cancel_requested):
        self.calls.append(f'failure:{message}')
        return ExecuteSpray.Result.VISION_FAILED, message

    def _return_to_observation(self):
        self.calls.append('return_to_observation')
        return True

    def _return_home(self, _cancel_requested):
        self.calls.append('return_home')
        return True

    def _request_motion_stop(self):
        self.calls.append('motion_stop')

    @staticmethod
    def _reset_fruit_tracking():
        pass

    @staticmethod
    def _select_target(_target_id):
        pass


def test_joint_preset_sequence_uses_recenter_then_visual_servo_before_spraying():
    task = _ClosedLoopSequenceHarness()
    request = SimpleNamespace(
        mission_id='mission-1', spray_duration=3.0,
        tree_hint=object())

    code, _message = task._run_sequence(
        request, lambda: False, lambda *_args: None)

    assert code == ExecuteSpray.Result.OK
    assert task.calls.count('spray') == 1
    assert 'request_aim' in task.calls
    assert 'lock:target-1' in task.calls
    assert 'recenter:target-1' in task.calls
    assert 'servo:target-1' in task.calls
    assert not any(call.startswith('fallback:') for call in task.calls)


def test_completed_queue_returns_home_without_an_extra_observation_scan():
    task = _ClosedLoopSequenceHarness()
    request = SimpleNamespace(
        mission_id='mission-1', spray_duration=3.0,
        tree_hint=object())

    code, _message = task._run_sequence(
        request, lambda: False, lambda *_args: None)

    assert code == ExecuteSpray.Result.OK
    assert task.calls.count('spray') == 1
    assert task.calls.count('return_to_observation') == 1
    assert task.calls[-1] == 'return_home'


def test_completed_tree_waits_once_before_final_home():
    class DelayedHarness(_ClosedLoopSequenceHarness):
        def _wait_post_spray_home_delay(
                self, sprayed, _cancel_requested, _feedback):
            self.calls.append(f'post_spray_wait:{sprayed}')
            return True

    task = DelayedHarness(targets=[
        _target('target-1', 620.0, 360.0),
        _target('target-2', 900.0, 200.0),
    ])
    request = SimpleNamespace(
        mission_id='mission-1', spray_duration=3.0,
        tree_hint=object())

    code, _message = task._run_sequence(
        request, lambda: False, lambda *_args: None)

    assert code == ExecuteSpray.Result.OK
    assert task.calls.count('spray') == 2
    assert task.calls.count('post_spray_wait:2') == 1
    assert task.calls[-2:] == ['post_spray_wait:2', 'return_home']


def test_no_disease_scans_all_fan_observations_then_returns_home():
    class NoDiseaseHarness(_ClosedLoopSequenceHarness):
        def __init__(self):
            super().__init__(targets=[])
            self._fan_observation_moves = 0

        def _wait_for_fruits(self, _cancel_requested):
            self._fruit_calls += 1
            return []

        def _move_to_next_fan_observation(self):
            self._fan_observation_moves += 1
            self.calls.append('next_fan_observation')
            return self._fan_observation_moves == 1

    task = NoDiseaseHarness()
    request = SimpleNamespace(
        mission_id='mission-1', spray_duration=3.0,
        tree_hint=object())

    code, message = task._run_sequence(
        request, lambda: False, lambda *_args: None)

    assert code == ExecuteSpray.Result.INSPECTED_NO_DISEASE
    assert 'no diseased fruit detected' in message
    assert task.calls.count('next_fan_observation') == 2
    assert 'spray' not in task.calls
    assert task.calls[-1] == 'return_home'


def test_first_nonempty_detection_locks_the_tree_target_set():
    initial = _target('target-1', 200.0, 200.0)
    discovered_later = _target('target-late', 900.0, 220.0)

    class InitialSetHarness(_ClosedLoopSequenceHarness):
        def _wait_for_fruits(self, _cancel_requested):
            self._fruit_calls += 1
            if self._fruit_calls == 1:
                return [initial]
            if self._fruit_calls == 2:
                return [initial, discovered_later]
            return []

    task = InitialSetHarness(targets=[initial])
    request = SimpleNamespace(
        mission_id='mission-1', spray_duration=3.0,
        tree_hint=object())

    code, message = task._run_sequence(
        request, lambda: False, lambda *_args: None)

    assert code == ExecuteSpray.Result.OK
    assert 'detected=1 sprayed=1 unresolved=0' in message
    assert 'lock:target-1' in task.calls
    assert 'lock:target-late' not in task.calls
    assert task.calls.count('spray') == 1


def test_frozen_ledger_blocks_reidentified_targets_after_each_spray():
    initial = [
        _target('initial-1', 620.0, 360.0),
        _target('initial-2', 900.0, 200.0),
    ]
    after_first_spray = [
        _target('new-1', 620.0, 360.0),
        _target('new-2', 900.0, 200.0),
    ]
    after_second_spray = [
        _target('newer-1', 620.0, 360.0),
        _target('newer-2', 900.0, 200.0),
    ]

    class TrackerChurnHarness(_ClosedLoopSequenceHarness):
        def _wait_for_fruits(self, _cancel_requested):
            self._fruit_calls += 1
            frames = [initial, after_first_spray, after_second_spray]
            self._visible_targets = frames[min(self._fruit_calls - 1, 2)]
            return self._visible_targets

        def _lock_target(self, target_id, _cancel_requested):
            self.calls.append(f'lock:{target_id}')
            return next(target for target in self._visible_targets
                        if target.target_id == target_id)

    task = TrackerChurnHarness(targets=initial)
    request = SimpleNamespace(
        mission_id='mission-1', spray_duration=3.0, tree_hint=object())

    code, message = task._run_sequence(
        request, lambda: False, lambda *_args: None)

    assert code == ExecuteSpray.Result.OK
    assert 'detected=2 sprayed=2 unresolved=0' in message
    assert task.calls.count('spray') == 2
    assert 'lock:new-2' in task.calls
    assert not any(call.startswith('lock:newer-') for call in task.calls)


def test_real_alignment_timeout_sprays_from_current_pose_then_rechecks_before_home():
    task = _ClosedLoopSequenceHarness(alignment_ok=False, fallback_enabled=True)
    request = SimpleNamespace(
        mission_id='mission-1', spray_duration=3.0,
        tree_hint=object())

    feedbacks = []
    code, _message = task._run_sequence(
        request, lambda: False, lambda *args: feedbacks.append(args))

    assert code == ExecuteSpray.Result.OK
    assert 'servo:target-1' in task.calls
    assert not any(call.startswith('fallback:') for call in task.calls)
    assert task.calls.count('spray') == 1
    assert task.calls.count('return_to_observation') == 1
    assert task.calls[-1] == 'return_home'
    assert any(item[0] == ExecuteSpray.Feedback.ALIGNING for item in feedbacks)


def test_servo_singularity_marks_only_that_target_unresolved_then_continues():
    class SingularityHarness(_ClosedLoopSequenceHarness):
        def _wait_for_fruits(self, _cancel_requested):
            self._fruit_calls += 1
            return self._observed_targets if self._fruit_calls <= 3 else []

        def _recover_to_next_observation(
                self, _cancel_requested, _feedback, _excluded_indices=None):
            self.calls.append('next_observation')
            return True, True

        @staticmethod
        def _rewind_for_untried_observation(_attempt):
            pass

        @staticmethod
        def _alignment_retry_allowed(attempt_count):
            return attempt_count < 2

    first = _target('target-1', 620.0, 360.0)
    second = _target('target-2', 900.0, 200.0)
    task = SingularityHarness(
        alignment_ok=True, fallback_enabled=True,
        alignment_code=AlignTarget.Result.SERVO_SINGULARITY,
        targets=[first, second], failed_target_ids={'target-1'})
    request = SimpleNamespace(
        mission_id='mission-1', spray_duration=3.0,
        tree_hint=object())

    code, _message = task._run_sequence(
        request, lambda: False, lambda *_args: None)

    assert code == ExecuteSpray.Result.PARTIAL_SUCCESS
    assert task.calls.count('spray') == 1
    assert task.calls.index('next_observation') < task.calls.index('lock:target-2')
    assert task.calls[-1] == 'return_home'


def test_endpoint_fallback_rechecks_and_sprays_the_remaining_target_before_home():
    first = _target('target-1', 620.0, 360.0)
    second = _target('target-2', 900.0, 200.0)
    task = _ClosedLoopSequenceHarness(
        targets=[first, second], failed_target_ids={'target-1'})
    request = SimpleNamespace(
        mission_id='mission-1', spray_duration=3.0,
        tree_hint=object())

    code, message = task._run_sequence(
        request, lambda: False, lambda *_args: None)

    assert code == ExecuteSpray.Result.OK
    assert 'detected=2 sprayed=2 unresolved=0' in message
    assert task.calls.count('spray') == 2
    assert task.calls.index('return_to_observation') < task.calls.index('lock:target-2')
    assert task.calls[-1] == 'return_home'


def test_servo_actuation_stall_sprays_current_pose_then_continues_queue():
    first = _target('target-1', 620.0, 360.0)
    second = _target('target-2', 900.0, 200.0)
    task = _ClosedLoopSequenceHarness(
        targets=[first, second], failed_target_ids={'target-1'},
        alignment_code=AlignTarget.Result.SERVO_ACTUATION_STALL)
    request = SimpleNamespace(
        mission_id='mission-1', spray_duration=3.0,
        tree_hint=object())

    code, message = task._run_sequence(
        request, lambda: False, lambda *_args: None)

    assert code == ExecuteSpray.Result.OK
    assert 'detected=2 sprayed=2 unresolved=0' in message
    assert task.calls.count('spray') == 2
    assert task.calls.index('return_to_observation') < task.calls.index('lock:target-2')
    assert task.calls[-1] == 'return_home'


def test_servo_safety_stop_locks_without_spraying():
    task = _ClosedLoopSequenceHarness(
        alignment_ok=False, fallback_enabled=True,
        alignment_code=AlignTarget.Result.SERVO_SAFETY_STOP)
    request = SimpleNamespace(
        mission_id='mission-1', spray_duration=3.0,
        tree_hint=object())

    code, _message = task._run_sequence(
        request, lambda: False, lambda *_args: None)

    assert code == ExecuteSpray.Result.LOCKED
    assert 'motion_stop' in task.calls
    assert 'spray' not in task.calls


def test_real_joint_limit_recenter_rejection_falls_back_to_safe_pose_spray():
    task = _ClosedLoopSequenceHarness(recenter_ok=False, fallback_enabled=True)
    request = SimpleNamespace(
        mission_id='mission-1', spray_duration=3.0,
        tree_hint=object())

    code, _message = task._run_sequence(
        request, lambda: False, lambda *_args: None)

    assert code == ExecuteSpray.Result.OK
    assert 'recenter:target-1' in task.calls
    assert 'fallback:target-1' in task.calls
    assert not any(call.startswith('servo:') for call in task.calls)
    assert task.calls.count('spray') == 1


class _ObservationHarness:
    _move_to_next_observation = SprayTask._move_to_next_observation
    _execute_candidate_motion = SprayTask._execute_candidate_motion

    def __init__(self):
        self._observation_candidates = [
            SimpleNamespace(
                candidate_id='bad-plan', distance_m=1.10,
                camera_position=(0.0, 0.0, 0.50),
                tool_position=(1.10, 0.0, 0.0),
                tool_quat=(0.0, 0.0, 0.0, 1.0),
                rejection_reason='', ik_joints=(0.0,) * 6,
                condition_number=5.0, min_joint_margin_rad=0.4,
                joint_motion_norm=0.1, visible=True,
                camera_height_m=1.5, azimuth_deg=0.0),
            SimpleNamespace(
                candidate_id='safe', distance_m=1.20,
                camera_position=(0.0, 0.0, 0.50),
                tool_position=(1.20, 0.0, 0.0),
                tool_quat=(0.0, 0.0, 0.0, 1.0),
                rejection_reason='', ik_joints=(0.0,) * 6,
                condition_number=5.0, min_joint_margin_rad=0.4,
                joint_motion_norm=0.1, visible=True,
                camera_height_m=1.5, azimuth_deg=0.0),
        ]
        self._observation_candidate_index = -1
        self._state_mutex = threading.Lock()
        self._joint_state_sequence = 0
        self._observation_distance = None
        self._observation_pose = None
        self._observation_config = {
            'position_tolerance_m': 0.01,
            'orientation_tolerance_rad': 0.01,
        }
        self.moves = []
        self._base_frame = 'base'
        self.arm_joint_names = ('joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6')
        self._observation_optimizer = SimpleNamespace(
            evaluate_ik=lambda candidate, *_args: candidate)
        self.arm = self

    def plan_pose(self, position, *_args, **_kwargs):
        self.moves.append(position[0])
        return None if position[0] == 1.10 else SimpleNamespace(position=position)

    @staticmethod
    def trajectory_final_positions(_trajectory, _joint_names):
        return (0.0,) * 6

    @staticmethod
    def execute_trajectory(_trajectory):
        return True

    @staticmethod
    def _wait_for_joint_state(*_args):
        return (0.0,) * 6

    def _aborted(self, _cancel_requested):
        return False

    def get_logger(self):
        return SimpleNamespace(info=lambda *_args: None, warn=lambda *_args: None)


def test_observation_candidates_advance_and_skip_planning_failures():
    task = _ObservationHarness()
    assert task._move_to_next_observation()
    assert task.moves == [1.10, 1.20]
    assert task._observation_candidate_index == 1
    assert task._observation_distance == 1.20


def test_observation_recovery_skips_indices_already_used_by_target():
    task = _ObservationHarness()

    assert task._move_to_next_observation({0})

    assert task.moves == [1.20]
    assert task._observation_candidate_index == 1


def test_tree_hint_validation_is_a_static_geometry_guard():
    valid = SimpleNamespace(
        header=SimpleNamespace(frame_id='map'),
        point=SimpleNamespace(x=1.0, y=2.0, z=3.0),
    )
    invalid = SimpleNamespace(
        header=SimpleNamespace(frame_id=''),
        point=SimpleNamespace(x=1.0, y=2.0, z=3.0),
    )

    assert SprayTask._hint_available(valid)
    assert not SprayTask._hint_available(invalid)


def test_feedback_callback_is_static_and_publishes_action_feedback():
    published = []
    handle = SimpleNamespace(publish_feedback=published.append)

    SprayTask._feedback(
        handle, ExecuteSpray.Feedback.MOVING_TO_OBSERVE,
        0.25, 'computing look-at pose')

    assert len(published) == 1
    assert published[0].phase == ExecuteSpray.Feedback.MOVING_TO_OBSERVE
    assert published[0].progress == 0.25
    assert published[0].phase_text == 'computing look-at pose'


class _RetryHarness:
    _alignment_retry_allowed = SprayTask._alignment_retry_allowed
    _is_recoverable_alignment_code = SprayTask._is_recoverable_alignment_code

    def get_parameter(self, _name):
        return SimpleNamespace(value=2)


def test_alignment_retry_is_bounded_by_configured_attempts():
    task = _RetryHarness()
    assert task._alignment_retry_allowed(1)
    assert not task._alignment_retry_allowed(2)


def test_servo_safety_stop_is_not_recoverable_alignment_code():
    assert not _RetryHarness._is_recoverable_alignment_code(
        AlignTarget.Result.SERVO_SAFETY_STOP)


def test_servo_actuation_stall_is_recoverable_alignment_code():
    assert _RetryHarness._is_recoverable_alignment_code(
        AlignTarget.Result.SERVO_ACTUATION_STALL)


class _AlignmentFallbackHarness:
    _alignment_fallback_target = SprayTask._alignment_fallback_target

    def __init__(self, *, enabled=True, locked=False, return_ok=True,
                 preflight_ok=True, confirmed=True):
        self._spray_on_alignment_failure = enabled
        self.state = SimpleNamespace(locked=locked)
        self._state_mutex = threading.Lock()
        self._joint_state_sequence = 4
        self.return_ok = return_ok
        self.preflight_ok = preflight_ok
        self.confirmed = confirmed
        self.calls = []
        self.target = _target('target-1', 640.0, 360.0)

    @staticmethod
    def _aborted(_cancel_requested):
        return False

    def _return_to_observation(self):
        self.calls.append('return_to_observation')
        return self.return_ok

    def _wait_for_joint_state(self, *, after_sequence):
        self.calls.append(f'fresh_joint_state:{after_sequence}')
        return (0.0,) * 6

    def _motion_preflight(self, target, joints, **kwargs):
        self.calls.append(f'preflight:{target.target_id}:{len(joints)}')
        assert kwargs['stage'] == 'FALLBACK'
        return self.preflight_ok, 'joint_limit_margin' if not self.preflight_ok else ''

    def _reset_target_confirmation(self, target_id):
        self.calls.append(f'reset:{target_id}')

    def _select_target(self, target_id):
        self.calls.append(f'select:{target_id}')

    def _set_inference_mode(self, mode):
        self.calls.append(f'mode:{mode}')

    def _wait_for_target_confirmation(
            self, target_id, _cancel_requested, *, require_workspace):
        self.calls.append(f'confirm:{target_id}:{require_workspace}')
        return self.confirmed

    def _latest_target(self):
        return self.target


def test_alignment_fallback_requires_safe_observation_and_fresh_target():
    task = _AlignmentFallbackHarness()

    target, reason = task._alignment_fallback_target(
        task.target, lambda: False)

    assert reason == ''
    assert target == task.target
    assert task.calls == [
        'return_to_observation', 'fresh_joint_state:4',
        'preflight:target-1:6', 'reset:target-1', 'select:target-1',
        'mode:target', 'confirm:target-1:False',
    ]


def test_alignment_fallback_never_sprays_when_locked_or_preflight_fails():
    disabled = _AlignmentFallbackHarness(enabled=False)
    target, reason = disabled._alignment_fallback_target(
        disabled.target, lambda: False)
    assert target is None
    assert reason == 'alignment fallback is disabled'
    assert disabled.calls == []

    locked = _AlignmentFallbackHarness(locked=True)
    target, reason = locked._alignment_fallback_target(
        locked.target, lambda: False)
    assert target is None
    assert reason == 'motion is canceled or locked'
    assert locked.calls == []

    unsafe = _AlignmentFallbackHarness(preflight_ok=False)
    target, reason = unsafe._alignment_fallback_target(
        unsafe.target, lambda: False)
    assert target is None
    assert reason == 'joint_limit_margin'
    assert not any(call.startswith('confirm:') for call in unsafe.calls)


def test_direction_divergence_can_use_safe_pose_fallback_but_singularity_cannot():
    assert SprayTask._alignment_code_allows_fallback(AlignTarget.Result.TIMEOUT)
    assert SprayTask._alignment_code_allows_fallback(AlignTarget.Result.TARGET_STALE)
    assert SprayTask._alignment_code_allows_fallback(
        AlignTarget.Result.SERVO_DIRECTION_DIVERGENCE)
    assert not SprayTask._alignment_code_allows_fallback(
        AlignTarget.Result.SERVO_SINGULARITY)
    assert not SprayTask._alignment_code_allows_fallback(
        AlignTarget.Result.SERVO_SAFETY_STOP)
    assert not SprayTask._alignment_code_allows_endpoint_spray(
        AlignTarget.Result.SERVO_SINGULARITY)


def test_target_recenter_uses_the_same_desired_spray_pixel_as_visual_servo():
    assert not target_requires_recenter(
        680.0, 388.0, 680.0, 388.0, 48.0)
    assert target_requires_recenter(
        730.0, 430.0, 680.0, 388.0, 48.0)
    assert target_requires_recenter(
        641.2, 389.2, 640.0, 388.0, 1.5)


class _TargetStateHarness:
    _on_selected_target = SprayTask._on_selected_target
    _lock_target = SprayTask._lock_target
    _reset_target_confirmation = SprayTask._reset_target_confirmation
    _latest_target = SprayTask._latest_target
    _active_aim_pixel = SprayTask._active_aim_pixel

    def __init__(self, target_id='fruit-1'):
        self._vision_mutex = threading.Lock()
        self._target_confirmation_id = target_id
        self._target_valid_frames = 0
        self._target_confirmation_frames = 0
        self._target_workspace_stable_since = None
        self._target_workspace_last_seen = None
        self._target_workspace_anchor = None
        self._target_workspace_currently_valid = False
        self._latest_selected_target = None
        self._active_aim = (640.0, 388.0, 1280, 720, 1.30)
        self._recenter_config = {
            'trigger_px': 48.0,
            'servo_entry_px': 48.0,
            'post_stable_sec': 0.50,
            'post_max_drift_px': 0.75,
            'post_max_gap_sec': 0.25,
            'post_min_confidence': 0.30,
        }

        self.events = []

    def _select_target(self, target_id):
        self.events.append(('select', target_id))

    def _set_inference_mode(self, mode):
        self.events.append(('mode', mode))

    def _wait_for_target_confirmation(
            self, target_id, _cancel_requested, *, require_workspace):
        assert not require_workspace
        self._latest_selected_target = _target(target_id, 777.0, 333.0)
        return True


class _TargetCallbackHarness(_TargetStateHarness):
    pass


def _target_message(center_u, center_v, *, valid=True, confidence=0.8):
    return SimpleNamespace(
        valid=valid, target_id='fruit-1', confidence=confidence,
        center_u=center_u, center_v=center_v, width=20.0, height=22.0,
        image_width=1280, image_height=720)


def test_target_confirmation_uses_target2d_mask_aim_point():
    task = _TargetCallbackHarness()
    for _ in range(3):
        task._on_selected_target(_target_message(800.0, 390.0))

    assert task._target_valid_frames == 3
    assert task._target_confirmation_frames == 0
    assert task._latest_selected_target.center_u == 800.0
    assert task._latest_selected_target.center_v == 390.0

    for _ in range(3):
        task._on_selected_target(_target_message(650.0, 390.0))
    assert task._target_confirmation_frames == 3


def test_post_recenter_gate_requires_confidence_and_half_second_span(monkeypatch):
    task = _TargetCallbackHarness()
    times = iter([0.0, 0.20, 0.40, 0.51])
    monkeypatch.setattr(
        'wvcsc_arm_task.target_flow.time.monotonic', lambda: next(times))

    task._on_selected_target(
        _target_message(650.0, 390.0, confidence=0.29))
    assert task._target_confirmation_frames == 0
    assert task._target_valid_frames == 1

    for _ in range(4):
        task._on_selected_target(
            _target_message(650.0, 390.0, confidence=0.30))
    assert task._target_confirmation_frames == 4
    assert math.isclose(
        task._target_workspace_last_seen -
        task._target_workspace_stable_since,
        0.51)


def test_post_recenter_gate_tolerates_one_short_invalid_frame(monkeypatch):
    task = _TargetCallbackHarness()
    times = iter([1.0, 1.1, 1.2])
    monkeypatch.setattr(
        'wvcsc_arm_task.target_flow.time.monotonic', lambda: next(times))
    task._on_selected_target(_target_message(650.0, 390.0))
    task._on_selected_target(_target_message(650.0, 390.0, valid=False))
    task._on_selected_target(_target_message(650.0, 390.0))

    assert task._target_confirmation_frames == 2
    assert task._target_workspace_stable_since == 1.0
    assert task._target_workspace_currently_valid


def test_post_recenter_gate_resets_after_sustained_invalid_gap(monkeypatch):
    task = _TargetCallbackHarness()
    times = iter([1.0, 1.3, 1.4])
    monkeypatch.setattr(
        'wvcsc_arm_task.target_flow.time.monotonic', lambda: next(times))
    task._on_selected_target(_target_message(650.0, 390.0))
    task._on_selected_target(_target_message(650.0, 390.0, valid=False))
    task._on_selected_target(_target_message(650.0, 390.0))

    assert task._target_confirmation_frames == 1
    assert task._target_workspace_stable_since == 1.4


def test_post_recenter_gate_restarts_when_aim_point_keeps_drifting(monkeypatch):
    task = _TargetCallbackHarness()
    times = iter([0.0, 0.2, 0.4, 0.6])
    monkeypatch.setattr(
        'wvcsc_arm_task.target_flow.time.monotonic', lambda: next(times))

    task._on_selected_target(_target_message(640.0, 388.0))
    task._on_selected_target(_target_message(640.4, 388.0))
    task._on_selected_target(_target_message(640.8, 388.0))
    task._on_selected_target(_target_message(641.2, 388.0))

    assert task._target_confirmation_frames == 2
    assert task._target_workspace_anchor == (640.8, 388.0)
    assert math.isclose(task._target_workspace_stable_since, 0.4)


class _TargetLockHarness(_TargetStateHarness):
    def __init__(self):
        super().__init__(target_id='')


def test_target_is_locked_before_camera_motion_and_returns_mask_aim():
    task = _TargetLockHarness()
    locked = task._lock_target('fruit-1', lambda: False)

    assert task.events == [('select', 'fruit-1'), ('mode', 'target')]
    assert locked.center_u == 777.0
    assert locked.center_v == 333.0


class _RecenterSafetyHarness:
    _move_to_recentered_pose = SprayTask._move_to_recentered_pose
    _execute_candidate_motion = SprayTask._execute_candidate_motion

    def __init__(self):
        self.arm = self
        self._state_mutex = threading.Lock()
        self._joint_state_sequence = 0
        self._base_frame = 'base'
        self.arm_joint_names = ('joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6')
        self.planning_called = False

    def compute_ik(self, *_args):
        return None

    def plan_pose(self, *_args, **_kwargs):
        self.planning_called = True


def test_target_recenter_rejects_collision_ik_before_planning_or_servo():
    task = _RecenterSafetyHarness()
    candidate = ObservationCandidate(
        candidate_id='recenter', distance_m=1.2, camera_height_m=1.5,
        azimuth_deg=0.0, camera_position=(1.0, 0.0, 1.0),
        camera_quat=(0.0, 0.0, 0.0, 1.0), tool_position=(1.0, 0.0, 1.0),
        tool_quat=(0.0, 0.0, 0.0, 1.0), visible=True, visible_margin_px=1.0)
    assert not task._move_to_recentered_pose(candidate, (0.0,) * 6)
    assert candidate.rejection_reason == 'collision_ik_failed'
    assert not task.planning_called


class _CameraPoseHarness:
    _current_camera_pose = SprayTask._current_camera_pose

    def __init__(self):
        self._base_frame = 'alicia_base_link'
        self._camera_frame = 'camera_color_optical_frame'
        self.requests = []
        self._tf_buffer = self

    def lookup_transform(self, target_frame, source_frame, _time):
        self.requests.append((target_frame, source_frame))
        return SimpleNamespace(transform=SimpleNamespace(
            translation=SimpleNamespace(x=0.1, y=-0.2, z=0.3),
            rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        ))

    @staticmethod
    def get_logger():
        return SimpleNamespace(debug=lambda *_args: None, warn=lambda *_args: None)


def test_recenter_uses_actual_camera_tf_as_its_geometric_feedback():
    task = _CameraPoseHarness()

    assert task._current_camera_pose() == (
        (0.1, -0.2, 0.3), (0.0, 0.0, 0.0, 1.0))
    assert task.requests == [('alicia_base_link', 'camera_color_optical_frame')]


class _RecenterAttemptHarness:
    _recenter_target = SprayTask._recenter_target
    _move_recenter_step = SprayTask._move_recenter_step
    _active_aim_pixel = SprayTask._active_aim_pixel
    _motion_preflight = SprayTask._motion_preflight
    _servo_handoff_preflight = SprayTask._servo_handoff_preflight
    _confirm_servo_handoff = SprayTask._confirm_servo_handoff

    def __init__(self):
        self._observation_candidate_index = 0
        self._observation_candidates = [SimpleNamespace(
            candidate_id='observe', distance_m=1.2, camera_height_m=1.5,
            azimuth_deg=0.0, camera_position=(1.0, 0.0, 1.0),
            camera_quat=(0.0, 0.0, 0.0, 1.0),
            tool_position=(1.0, 0.0, 1.0),
            tool_quat=(0.0, 0.0, 0.0, 1.0),
            observation_mode='ik', joint_positions=(), rejection_reason='')]
        self._observation_optimizer = SimpleNamespace(
            evaluate_ik=lambda candidate, *_args: candidate)
        self._camera_mount = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
        self._active_aim = (640.0, 388.0, 1280, 720, 1.30)
        self._recenter_config = {
            'trigger_px': 48.0,
            'servo_entry_px': 48.0,
            'max_angle_deg': 18.0,
            'refine_goal_px': 8.0, 'max_iterations': 1,
            'residual_candidates_px': (
                12.0, 16.0, 24.0, 8.0, 32.0, 40.0, 3.0, 1.0, 0.0),
        }

    @staticmethod
    def _wait_for_observation_inputs():
        return (500.0, 500.0, 640.0, 360.0, 1280, 720), (0.0,) * 6

    @staticmethod
    def _current_camera_pose():
        return (1.0, 0.0, 1.0), (0.0, 0.0, 0.0, 1.0)


def test_recenter_angle_rejection_is_recoverable_and_not_retried_at_same_view():
    task = _RecenterAttemptHarness()
    attempt = TargetAttempt(_target('fruit-1', 1000.0, 360.0))
    ok, message = task._recenter_target(attempt.target, attempt, lambda: False)
    assert not ok
    assert 'angle exceeds limit' in message
    assert attempt.recentered_observation_indices == {0}
    ok, message = task._recenter_target(attempt.target, attempt, lambda: False)
    assert not ok
    assert message == 'target recenter already used at this observation'


class _PartialRecenterHarness(_RecenterAttemptHarness):
    def __init__(self):
        super().__init__()
        self.trials = []

    def _move_to_recentered_pose(self, candidate, _joints):
        self.trials.append(candidate.candidate_id)
        candidate.condition_number = 10.0
        candidate.min_joint_margin_rad = 0.20
        if candidate.candidate_id.endswith('_r16'):
            return True
        candidate.rejection_reason = 'joint_limit_margin'
        return False

    @staticmethod
    def _aborted(_cancel_requested):
        return False

    @staticmethod
    def _reset_target_confirmation(_target_id):
        pass

    @staticmethod
    def _wait_for_target_confirmation(
            _target_id, _cancel_requested, *, require_workspace):
        return True

    @staticmethod
    def _latest_target():
        return _target('fruit-1', 650.0, 390.0)

    @staticmethod
    def get_logger():
        return SimpleNamespace(info=lambda *_args: None)


def test_recenter_uses_a_larger_residual_when_precise_rotation_lacks_margin():
    task = _PartialRecenterHarness()
    attempt = TargetAttempt(_target('fruit-1', 740.0, 360.0))

    ok, message = task._recenter_target(
        attempt.target, attempt, lambda: False)

    assert ok
    assert message == 'target reconfirmed after recenter'
    assert task.trials == [
        'observe_target_fruit-1_r12',
        'observe_target_fruit-1_r16',
    ]


def test_recenter_residual_inside_angular_limit_is_recentered():
    task = _PartialRecenterHarness()
    task._recenter_config['trigger_px'] = 1.5
    attempt = TargetAttempt(_target('fruit-1', 740.0, 360.0))

    ok, message = task._recenter_target(
        attempt.target, attempt, lambda: False)

    assert ok
    assert message == 'target reconfirmed after recenter'


def test_large_residual_runs_coarse_recenter_before_servo_handoff():
    task = _PartialRecenterHarness()
    task._recenter_config.update({
        'trigger_px': 48.0,
        'servo_entry_px': 48.0,
        'max_iterations': 1,
    })
    task._latest_target = lambda: _target('fruit-1', 650.0, 390.0)
    attempt = TargetAttempt(_target('fruit-1', 750.0, 388.0))

    ok, message = task._recenter_target(
        attempt.target, attempt, lambda: False)

    assert ok
    assert message == 'target reconfirmed after recenter'
    assert task.trials == [
        'observe_target_fruit-1_r12',
        'observe_target_fruit-1_r16',
    ]


def test_coarse_recenter_rejects_current_joint_limit_margin():
    task = _PartialRecenterHarness()
    task._recenter_config.update({
        'trigger_px': 48.0,
        'servo_entry_px': 48.0,
    })

    def reject(candidate, *_args):
        candidate.rejection_reason = 'joint_limit_margin'
        return candidate

    task._observation_optimizer = SimpleNamespace(evaluate_ik=reject)
    attempt = TargetAttempt(_target('fruit-1', 750.0, 388.0))

    ok, message = task._recenter_target(
        attempt.target, attempt, lambda: False)

    assert not ok
    assert message == 'Servo preflight rejected: joint_limit_margin'
    assert task.trials == [
        'observe_target_fruit-1_r12',
        'observe_target_fruit-1_r16',
    ]


class _NoRecenterDriftHarness:
    _recenter_target = SprayTask._recenter_target
    _active_aim_pixel = SprayTask._active_aim_pixel
    _motion_preflight = SprayTask._motion_preflight
    _servo_handoff_preflight = SprayTask._servo_handoff_preflight
    _confirm_servo_handoff = SprayTask._confirm_servo_handoff

    def __init__(self):
        self._recenter_config = {
            'trigger_px': 48.0,
            'servo_entry_px': 48.0,
        }
        self._active_aim = (640.0, 388.0, 1280, 720, 1.30)
        self._observation_candidate_index = 0
        self._observation_candidates = [SimpleNamespace(
            candidate_id='observe', distance_m=1.2, camera_height_m=1.5,
            azimuth_deg=0.0, camera_position=(1.0, 0.0, 1.0),
            camera_quat=(0.0, 0.0, 0.0, 1.0),
            tool_position=(1.0, 0.0, 1.0),
            tool_quat=(0.0, 0.0, 0.0, 1.0),
            observation_mode='ik', joint_positions=(), rejection_reason='')]
        self._observation_optimizer = SimpleNamespace(
            evaluate_ik=lambda candidate, *_args: candidate)

    @staticmethod
    def _wait_for_observation_inputs():
        return (500.0, 500.0, 640.0, 360.0, 1280, 720), (0.0,) * 6

    @staticmethod
    def _wait_for_target_confirmation(
            _target_id, _cancel_requested, *, require_workspace):
        assert not require_workspace
        return True

    @staticmethod
    def _latest_target():
        # Desired point is (640, 388); the confirmed target drifted to 644.
        return _target('fruit-1', 644.0, 388.0)

    @staticmethod
    def get_logger():
        return SimpleNamespace(info=lambda *_args: None)


def test_small_residual_inside_workspace_is_sent_to_visual_servo():
    task = _NoRecenterDriftHarness()
    attempt = TargetAttempt(_target('fruit-1', 641.0, 388.0))

    ok, message = task._recenter_target(
        attempt.target, attempt, lambda: False)

    assert ok
    assert message == 'target reconfirmed for visual servo'


def test_large_residual_inside_workspace_is_sent_to_visual_servo():
    task = _NoRecenterDriftHarness()
    task._latest_target = lambda: _target('fruit-1', 652.0, 427.0)
    attempt = TargetAttempt(_target('fruit-1', 652.0, 427.0))

    ok, message = task._recenter_target(
        attempt.target, attempt, lambda: False)

    assert ok
    assert message == 'target reconfirmed for visual servo'


class _IterativeRecenterHarness(_PartialRecenterHarness):
    def __init__(self):
        super().__init__()
        self._recenter_config['max_iterations'] = 2
        self._actual_camera_poses = iter([
            ((1.10, 0.0, 1.0), (0.0, 0.0, 0.0, 1.0)),
            ((1.20, 0.0, 1.0), (0.0, 0.0, 0.0, 1.0)),
        ])
        self.planned_camera_positions = []
        self._latest_targets = iter([
            _target('fruit-1', 660.0, 388.0),
            _target('fruit-1', 645.0, 388.0),
        ])

    def _current_camera_pose(self):
        return next(self._actual_camera_poses)

    def _move_to_recentered_pose(self, candidate, _joints):
        self.trials.append(candidate.candidate_id)
        self.planned_camera_positions.append(candidate.camera_position)
        candidate.condition_number = 10.0
        candidate.min_joint_margin_rad = 0.50
        return True

    def _latest_target(self):
        return next(self._latest_targets)


def test_recenter_refines_again_when_mask_aim_moves_after_first_motion():
    task = _IterativeRecenterHarness()
    attempt = TargetAttempt(_target('fruit-1', 740.0, 360.0))

    ok, _message = task._recenter_target(
        attempt.target, attempt, lambda: False)

    assert ok
    assert len(task.trials) == 2
    assert task.trials[0].endswith('_r12')
    assert task.trials[1].endswith('_r12_refine1')
    assert task.planned_camera_positions == [
        (1.10, 0.0, 1.0),
        (1.20, 0.0, 1.0),
    ]


class _FiveStepRecenterHarness(_IterativeRecenterHarness):
    def __init__(self):
        super().__init__()
        self._recenter_config.update({
            'trigger_px': 3.0,
            'refine_goal_px': 1.0,
            'max_iterations': 5,
        })
        self._actual_camera_poses = iter([
            ((1.10, 0.0, 1.0), (0.0, 0.0, 0.0, 1.0)),
            ((1.20, 0.0, 1.0), (0.0, 0.0, 0.0, 1.0)),
            ((1.30, 0.0, 1.0), (0.0, 0.0, 0.0, 1.0)),
            ((1.40, 0.0, 1.0), (0.0, 0.0, 0.0, 1.0)),
        ])
        self._latest_targets = iter([
            _target('fruit-1', 660.0, 388.0),
            _target('fruit-1', 650.0, 388.0),
            _target('fruit-1', 644.1, 388.0),
            _target('fruit-1', 640.5, 388.0),
        ])


def test_recenter_can_use_two_extra_safe_refinements_for_small_residuals():
    task = _FiveStepRecenterHarness()
    attempt = TargetAttempt(_target('fruit-1', 740.0, 388.0))

    ok, message = task._recenter_target(
        attempt.target, attempt, lambda: False)

    assert ok
    assert message == 'target reconfirmed after recenter'
    assert len(task.trials) == 4


class _PostRecenterHarness:
    _recenter_target = SprayTask._recenter_target
    _move_recenter_step = SprayTask._move_recenter_step
    _active_aim_pixel = SprayTask._active_aim_pixel
    _motion_preflight = SprayTask._motion_preflight
    _servo_handoff_preflight = SprayTask._servo_handoff_preflight
    _confirm_servo_handoff = SprayTask._confirm_servo_handoff

    def __init__(self):
        self._observation_candidate_index = 0
        self._observation_candidates = [SimpleNamespace(
            candidate_id='observe', distance_m=1.2, camera_height_m=1.5,
            azimuth_deg=0.0, camera_position=(1.0, 0.0, 1.0),
            camera_quat=(0.0, 0.0, 0.0, 1.0),
            tool_position=(1.0, 0.0, 1.0),
            tool_quat=(0.0, 0.0, 0.0, 1.0),
            observation_mode='ik', joint_positions=(), rejection_reason='')]
        self._observation_optimizer = SimpleNamespace(
            evaluate_ik=lambda candidate, *_args: candidate)
        self._camera_mount = (
            (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
        self._active_aim = (640.0, 388.0, 1280, 720, 1.30)
        self._recenter_config = {
            'trigger_px': 48.0,
            'servo_entry_px': 48.0,
            'max_angle_deg': 18.0,
            'refine_goal_px': 8.0, 'max_iterations': 1,
            'residual_candidates_px': (
                12.0, 16.0, 24.0, 8.0, 32.0, 40.0, 3.0, 1.0, 0.0),
        }

    @staticmethod
    def _wait_for_observation_inputs():
        return (500.0, 500.0, 640.0, 360.0, 1280, 720), (0.0,) * 6

    @staticmethod
    def _current_camera_pose():
        return (1.0, 0.0, 1.0), (0.0, 0.0, 0.0, 1.0)

    @staticmethod
    def _move_to_recentered_pose(_candidate, _joints):
        return True

    @staticmethod
    def _aborted(_cancel_requested):
        return False

    @staticmethod
    def _reset_target_confirmation(_target_id):
        pass

    @staticmethod
    def _wait_for_target_confirmation(
            _target_id, _cancel_requested, *, require_workspace):
        return not require_workspace

    @staticmethod
    def _latest_target():
        return _target('fruit-1', 650.0, 390.0)

    @staticmethod
    def get_logger():
        return SimpleNamespace(info=lambda *_args: None)


def test_post_recenter_jitter_gate_does_not_block_servo_handoff():
    task = _PostRecenterHarness()
    attempt = TargetAttempt(_target('fruit-1', 740.0, 360.0))

    ok, message = task._recenter_target(
        attempt.target, attempt, lambda: False)

    assert ok
    assert message == 'target reconfirmed after recenter'


class _AlignHarness:
    _align_target = SprayTask._align_target
    _vision_timeout = 8.0
    _downstream_margin = 2.0
    _vision_client = object()

    def _run_downstream_action(self, *_args, **_kwargs):
        result = SimpleNamespace(
            success=False,
            error_code=AlignTarget.Result.SERVO_SINGULARITY,
            message='MoveIt Servo recoverable status 2')
        return SimpleNamespace(
            status=GoalStatus.STATUS_ABORTED, result=result), False, ''


def test_alignment_result_code_and_message_are_not_replaced_by_canceled_text():
    ok, canceled, code, message = _AlignHarness()._align_target(
        'mission-1', 'fruit-1',
        (640.0, 388.0, 1280, 720, 1.30), lambda: False)
    assert not ok
    assert not canceled
    assert code == AlignTarget.Result.SERVO_SINGULARITY
    assert message == 'MoveIt Servo recoverable status 2'


def test_alignment_feedback_callback_is_forwarded_to_the_action_client():
    callback = lambda _message: None
    task = _AlignHarness()
    task._run_downstream_action = lambda *_args, **kwargs: (
        setattr(task, 'feedback_callback', kwargs['feedback_callback']) or
        (SimpleNamespace(
            status=GoalStatus.STATUS_SUCCEEDED,
            result=SimpleNamespace(
                success=True, error_code=AlignTarget.Result.OK, message='ok')),
         False, ''))

    task._align_target(
        'mission-1', 'fruit-1',
        (640.0, 388.0, 1280, 720, 1.30), lambda: False,
        feedback_callback=callback)

    assert task.feedback_callback is callback


def test_new_target_can_wrap_to_an_untried_earlier_observation():
    task = SimpleNamespace(
        _observation_candidate_index=1,
        _observation_candidates=[object(), object()])
    attempt = SimpleNamespace(recentered_observation_indices={1})

    SprayTask._rewind_for_untried_observation(task, attempt)

    assert task._observation_candidate_index == -1


class _RecoveryScanHarness:
    def __init__(self, candidate_index, candidate_count):
        self._observation_candidate_index = candidate_index
        self._observation_candidates = [object()] * candidate_count
        self.selected = []
        self.modes = []
        self.reset_count = 0

    def _select_target(self, value):
        self.selected.append(value)

    def _set_inference_mode(self, value):
        self.modes.append(value)

    def _reset_fruit_tracking(self):
        self.reset_count += 1

    @staticmethod
    def get_logger():
        return SimpleNamespace(info=lambda *_args: None)


class _MissingTargetRecoveryHarness(_RecoveryScanHarness):
    _recover_missing_target = SprayTask._recover_missing_target
    _attempt_for = SprayTask._attempt_for
    _same_target = SprayTask._same_target
    _rewind_for_untried_observation = SprayTask._rewind_for_untried_observation

    def __init__(self):
        super().__init__(candidate_index=0, candidate_count=2)

    def get_parameter(self, name):
        return SimpleNamespace(value={
            'processed_iou_threshold': 0.30,
            'processed_center_distance_px': 40.0,
        }[name])

    def _recover_to_next_observation(
            self, _cancel_requested, _feedback, excluded_indices=None):
        assert excluded_indices == {0}
        self._observation_candidate_index = 1
        return True, True

def test_missing_target_searches_another_safe_view_before_unresolved():
    task = _MissingTargetRecoveryHarness()
    attempts = []
    target = _target('fruit-2', 800.0, 360.0)

    attempt, recovered, moved = task._recover_missing_target(
        target, None, attempts, lambda: False, lambda *_args: None)

    assert recovered and moved
    assert attempts == [attempt]
    assert attempt.target == target
    assert attempt.recentered_observation_indices == {0}
    assert task._observation_candidate_index == 1
    assert task.selected == ['']
    assert task.modes == ['idle']
    assert task.reset_count == 1
