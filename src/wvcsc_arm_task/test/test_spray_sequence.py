import threading
from types import SimpleNamespace

from action_msgs.msg import GoalStatus
from wvcsc_interfaces.action import AlignTarget

from wvcsc_arm_task.spray_task import (
    FruitTarget, SprayTask, detection_candidates, spray_summary)


def _target(target_id, center_u, center_v, confidence=0.9):
    return FruitTarget(target_id, confidence, center_u, center_v, 40.0, 40.0)


class _QueueHarness:
    _queue = SprayTask._queue
    _attempt_for = SprayTask._attempt_for
    _same_target = SprayTask._same_target
    _mark_unresolved = SprayTask._mark_unresolved

    def get_parameter(self, name):
        return SimpleNamespace(value={
            'processed_iou_threshold': 0.30,
            'processed_center_distance_px': 40.0,
            'image_width': 1280,
            'image_height': 720,
        }[name])


def test_queue_excludes_processed_target_by_geometry_not_only_tracker_id():
    task = _QueueHarness()
    processed = _target('old-id', 640.0, 360.0)
    same_fruit_new_id = _target('new-id', 650.0, 360.0)
    other = _target('other', 800.0, 360.0)
    queue = task._queue([same_fruit_new_id, other], [processed])
    assert queue == [other]


def test_queue_prefers_the_fruit_nearest_the_image_center():
    task = _QueueHarness()
    near = _target('near', 650.0, 365.0, confidence=0.80)
    far = _target('far', 900.0, 600.0, confidence=0.99)
    assert task._queue([far, near], []) == [near, far]


def test_iou_is_zero_for_disjoint_targets():
    assert _target('a', 100.0, 100.0).iou(_target('b', 300.0, 300.0)) == 0.0


def test_diseased_fruit_below_point_one_never_enters_the_queue():
    def detection(score):
        return SimpleNamespace(
            id='fruit',
            results=[SimpleNamespace(hypothesis=SimpleNamespace(
                class_id='diseased_fruit', score=score))],
            bbox=SimpleNamespace(
                center=SimpleNamespace(position=SimpleNamespace(x=640.0, y=360.0)),
                size_x=40.0, size_y=40.0),
        )

    assert detection_candidates(
        SimpleNamespace(detections=[detection(0.099)]), 'diseased_fruit', 0.10) == []
    assert len(detection_candidates(
        SimpleNamespace(detections=[detection(0.10)]), 'diseased_fruit', 0.10)) == 1


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
    assert spray_summary(1, 0, len(exhausted), 1) == (
        'detected=1 sprayed=0 unresolved=1 alignment_failures=1')


class _ObservationHarness:
    _move_to_next_observation = SprayTask._move_to_next_observation
    _OBSERVATION_POSITION_TOLERANCE = SprayTask._OBSERVATION_POSITION_TOLERANCE
    _OBSERVATION_ORIENTATION_TOLERANCE = SprayTask._OBSERVATION_ORIENTATION_TOLERANCE

    def __init__(self):
        self._observation_candidates = [
            SimpleNamespace(
                candidate_id='bad-plan', distance_m=1.10,
                tool_position=(1.10, 0.0, 0.0),
                tool_quat=(0.0, 0.0, 0.0, 1.0),
                rejection_reason='', ik_joints=(0.0,) * 6,
                condition_number=5.0, min_joint_margin_rad=0.4,
                joint_motion_norm=0.1, visible=True,
                camera_height_m=1.5, azimuth_deg=0.0),
            SimpleNamespace(
                candidate_id='safe', distance_m=1.20,
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

    def _publish_observation_debug(self, *_args, **_kwargs):
        pass

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


class _RetryHarness:
    _alignment_retry_allowed = SprayTask._alignment_retry_allowed

    def get_parameter(self, _name):
        return SimpleNamespace(value=2)


def test_alignment_retry_is_bounded_by_configured_attempts():
    task = _RetryHarness()
    assert task._alignment_retry_allowed(1)
    assert not task._alignment_retry_allowed(2)


class _AlignHarness:
    _align_target = SprayTask._align_target
    _vision_timeout = 8.0
    _downstream_margin = 2.0
    _vision_client = object()

    def _run_downstream_action(self, *_args):
        result = SimpleNamespace(
            success=False,
            error_code=AlignTarget.Result.SERVO_SINGULARITY,
            message='MoveIt Servo recoverable status 2')
        return SimpleNamespace(
            status=GoalStatus.STATUS_ABORTED, result=result), False, ''


def test_alignment_result_code_and_message_are_not_replaced_by_canceled_text():
    ok, canceled, code, message = _AlignHarness()._align_target(
        'mission-1', 'tree-1', 'fruit-1', lambda: False)
    assert not ok
    assert not canceled
    assert code == AlignTarget.Result.SERVO_SINGULARITY
    assert message == 'MoveIt Servo recoverable status 2'
