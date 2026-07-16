from types import SimpleNamespace

from wvcsc_arm_task.spray_task import FruitTarget, SprayTask


def _target(target_id, center_u, center_v, confidence=0.9):
    return FruitTarget(target_id, confidence, center_u, center_v, 40.0, 40.0)


class _QueueHarness:
    _queue = SprayTask._queue

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
