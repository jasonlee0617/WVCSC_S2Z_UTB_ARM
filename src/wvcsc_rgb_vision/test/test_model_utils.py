import json
import numpy as np
import pytest
from std_msgs.msg import Header
from types import SimpleNamespace

from wvcsc_rgb_vision.model_utils import (
    FRUIT_CLASS_NAMES,
    TREE_CLASS_NAMES,
    canonical_class_name,
    validate_yolo_model,
)
from wvcsc_rgb_vision.two_stage_yolo import (
    PERCEPTION_DEBUG_DEFAULTS,
    Instance,
    Track,
    TwoStageYolo,
    deduplicate_instances,
    expanded_roi,
    perception_debug_due,
    perception_debug_json,
    track_matches,
    safest_mask_point,
)


def test_two_models_keep_independent_class_contracts():
    assert TREE_CLASS_NAMES == {0: 'tree'}
    assert FRUIT_CLASS_NAMES == {0: 'healthy_fruit', 1: 'diseased_fruit'}
    assert canonical_class_name(0, TREE_CLASS_NAMES) == 'tree'
    assert canonical_class_name(1, FRUIT_CLASS_NAMES) == 'diseased_fruit'


def test_deployment_weight_must_match_task_and_classes():
    model = type('Model', (), {
        'task': 'segment',
        'names': {0: 'healthy_fruit', 1: 'diseased_fruit'},
    })()
    validate_yolo_model(model, 'segment', FRUIT_CLASS_NAMES)
    model.names[1] = 'wrong_class'
    with pytest.raises(ValueError, match='contract mismatch'):
        validate_yolo_model(model, 'segment', FRUIT_CLASS_NAMES)


def test_fruit_tracks_survive_a_short_detector_dropout():
    node = object.__new__(TwoStageYolo)
    node._tracks = []
    node._next_target_number = 1
    node.get_parameter = lambda name: type('Parameter', (), {'value': {
        'track_iou_threshold': 0.30,
        'track_center_distance_px': 40.0,
        'track_max_missed_frames': 3,
    }[name]})()
    fruit = Instance('', 'diseased_fruit', 0.9, 10.0, 10.0, 30.0, 30.0, 20.0, 20.0)
    first = node._assign_track_ids([fruit])[0]
    node._assign_track_ids([])
    again = node._assign_track_ids([fruit])[0]
    assert first.target_id == again.target_id


def _target_selection_node(selected_id, reference):
    node = object.__new__(TwoStageYolo)
    node._selected_target_id = selected_id
    node._selected_target_reference = reference
    node.get_parameter = lambda name: SimpleNamespace(value={
        'track_iou_threshold': 0.30,
        'track_center_distance_px': 40.0,
        'target_reassociation_iou_margin': 0.10,
        'target_reassociation_distance_margin_px': 8.0,
        'target_equivalent_aim_distance_px': 8.0,
        'target_lock_ema_alpha': 0.35,
    }[name])
    return node


def test_selected_target_is_reassociated_when_its_tracker_id_changes():
    reference = Instance('fruit-1', 'diseased_fruit', 0.9, 10, 10, 30, 30, 20, 20)
    reassociated = Instance('fruit-9', 'diseased_fruit', 0.9, 12, 10, 32, 30, 22, 20)
    node = _target_selection_node('fruit-1', reference)

    target, reason, event = node._resolve_selected_target([reassociated])

    assert target.target_id == reassociated.target_id
    assert target.aim_u == pytest.approx(20.7)
    assert reason == 'none'
    assert event == 'target_reassociated'
    assert node._selected_target_reference == target


def test_repeated_selected_target_message_preserves_geometric_reference():
    reference = Instance(
        'fruit-9', 'diseased_fruit', 0.9, 12, 10, 32, 30, 22, 20)
    node = object.__new__(TwoStageYolo)
    node._selected_target_id = 'fruit-1'
    node._selected_target_reference = reference
    node._tracks = []
    node._last_target_state = ('target_valid',)

    node._on_selected_target(SimpleNamespace(data='fruit-1'))

    assert node._selected_target_reference is reference
    assert node._last_target_state == ('target_valid',)


def test_candidate_id_churn_keeps_the_published_logical_target_id():
    class Publisher:
        def __init__(self):
            self.messages = []

        def publish(self, message):
            self.messages.append(message)

    reference = Instance(
        'fruit-1', 'diseased_fruit', 0.9, 10, 10, 30, 30, 20, 20)
    node = _target_selection_node('fruit-1', reference)
    node._mission_id = 'mission-1'
    node._tree_id = 'tree-1'
    node._target_pub = Publisher()
    node._log_target_state = lambda *_args: None
    image = SimpleNamespace(
        header=Header(), width=1280, height=720)

    for candidate_id, left in [('fruit-9', 12.0), ('fruit-10', 14.0)]:
        candidate = Instance(
            candidate_id, 'diseased_fruit', 0.9,
            left, 10.0, left + 20.0, 30.0, left + 10.0, 20.0)
        target, reason, _event = node._publish_selected_target(
            image, [candidate])
        assert target.target_id == candidate_id
        assert reason == 'none'
        assert node._target_pub.messages[-1].target_id == 'fruit-1'
        assert node._target_pub.messages[-1].valid


def test_selected_target_refuses_ambiguous_reassociation():
    reference = Instance('fruit-1', 'diseased_fruit', 0.9, 10, 10, 30, 30, 20, 20)
    node = _target_selection_node('fruit-1', reference)
    candidates = [
        Instance('fruit-9', 'diseased_fruit', 0.9, 11, 10, 31, 30, 21, 20),
        Instance('fruit-10', 'diseased_fruit', 0.9, 9, 10, 29, 30, 11, 20),
    ]

    target, reason, event = node._resolve_selected_target(candidates)

    assert target is None
    assert reason == 'ambiguous_reassociation'
    assert event == 'target_invalid'


def test_selected_target_accepts_equivalent_segmentation_masks():
    reference = Instance('fruit-1', 'diseased_fruit', 0.9, 10, 10, 30, 30, 20, 20)
    node = _target_selection_node('fruit-1', reference)
    candidates = [
        Instance('fruit-9', 'diseased_fruit', 0.9, 11, 10, 31, 30, 21, 20),
        Instance('fruit-10', 'diseased_fruit', 0.9, 9, 10, 29, 30, 19, 20),
    ]

    target, reason, event = node._resolve_selected_target(candidates)

    assert target is not None
    assert target.target_id == 'fruit-10'
    assert reason == 'none'
    assert event == 'target_reassociated'


def test_stale_tracker_id_cannot_steal_the_geometry_locked_target():
    reference = Instance('fruit-1', 'diseased_fruit', 0.9, 10, 10, 30, 30, 20, 20)
    node = _target_selection_node('fruit-1', reference)
    near_replacement = Instance('fruit-9', 'diseased_fruit', 0.9, 11, 10, 31, 30, 21, 20)
    stale_id = Instance('fruit-1', 'diseased_fruit', 0.9, 200, 200, 220, 220, 210, 210)

    target, reason, event = node._resolve_selected_target([stale_id, near_replacement])

    assert reason == 'none'
    assert event == 'target_reassociated'
    assert target.target_id == 'fruit-9'
    assert 20.0 < target.aim_u < 21.0


def test_track_matching_is_one_to_one_when_detector_order_changes():
    left = Instance('fruit-1', 'diseased_fruit', 0.9, 10, 10, 30, 30, 20, 20)
    right = Instance('fruit-2', 'diseased_fruit', 0.9, 70, 10, 90, 30, 80, 20)
    reordered = [
        Instance('', 'diseased_fruit', 0.9, 72, 10, 92, 30, 82, 20),
        Instance('', 'diseased_fruit', 0.9, 12, 10, 32, 30, 22, 20),
    ]

    matches = track_matches(reordered, [Track(left), Track(right)], 0.30, 40.0)

    assert matches == {0: 1, 1: 0}


def test_perception_debug_payload_has_the_stable_schema_and_rate_limit():
    payload = json.loads(perception_debug_json(
        event='target_valid', target_valid=True, selected_target_id='fruit-1',
        candidate_target_id='fruit-9'))

    assert set(payload) == set(PERCEPTION_DEBUG_DEFAULTS)
    assert payload['event'] == 'target_valid'
    assert payload['target_valid'] is True
    assert payload['candidate_target_id'] == 'fruit-9'
    assert perception_debug_due(None, 1.0, 5.0)
    assert not perception_debug_due(1.0, 1.19, 5.0)
    assert perception_debug_due(1.0, 1.20, 5.0)


def test_roi_expansion_clips_to_image_bounds():
    assert expanded_roi(5, 5, 30, 30, 100, 80, 0.10) == (2, 2, 33, 33)


def test_mask_safe_point_is_inside_the_instance_polygon():
    polygon = np.array([[10, 10], [50, 10], [50, 50], [10, 50]], dtype=float)
    u, v = safest_mask_point(polygon, 64, 64)
    assert 20 <= u <= 40
    assert 20 <= v <= 40


def test_deduplication_keeps_highest_confidence_same_class_instance():
    best = Instance(
        '', 'diseased_fruit', 0.80, 10, 10, 30, 30, 20, 20)
    overlapping = Instance(
        '', 'diseased_fruit', 0.60, 12, 12, 32, 32, 22, 22)
    near_center = Instance(
        '', 'diseased_fruit', 0.70, 17, 17, 37, 37, 27, 27)

    assert deduplicate_instances(
        [overlapping, near_center, best]) == [best]


def test_fruit_inference_uses_stricter_nms_before_custom_deduplication():
    calls = []

    class Model:
        def __call__(self, _image, **kwargs):
            calls.append(kwargs)
            return [object()]

    stronger = Instance(
        '', 'diseased_fruit', 0.80, 10, 10, 30, 30, 20, 20)
    weaker = Instance(
        '', 'diseased_fruit', 0.50, 12, 12, 32, 32, 22, 22)
    node = object.__new__(TwoStageYolo)
    node._fruit_model = Model()
    node.get_parameter = lambda name: SimpleNamespace(value={
        'roi_padding': 0.0,
        'fruit_confidence': 0.10,
    }[name])
    node._seg_instances = lambda *_args: [weaker, stronger]
    tree = Instance('', 'tree', 1.0, 0, 0, 64, 64, 32, 32)

    result = node._fruit_instances(
        np.zeros((64, 64, 3), dtype=np.uint8), tree)

    assert calls == [{'verbose': False, 'conf': 0.10, 'iou': 0.45}]
    assert result == [stronger]


def test_deduplication_drops_ambiguous_cross_class_instance():
    healthy = Instance(
        '', 'healthy_fruit', 0.61, 10, 10, 30, 30, 20, 20)
    diseased = Instance(
        '', 'diseased_fruit', 0.65, 11, 11, 31, 31, 21, 21)

    assert deduplicate_instances([healthy, diseased]) == []


def test_deduplication_keeps_clear_cross_class_winner():
    healthy = Instance(
        '', 'healthy_fruit', 0.40, 10, 10, 30, 30, 20, 20)
    diseased = Instance(
        '', 'diseased_fruit', 0.65, 11, 11, 31, 31, 21, 21)

    assert deduplicate_instances([healthy, diseased]) == [diseased]


def test_visualization_labels_include_id_class_and_confidence():
    instance = Instance(
        'fruit-7', 'diseased_fruit', 0.937,
        10.4, 20.6, 30.4, 40.6, 20.0, 30.0)
    assert TwoStageYolo._label(instance) == (
        'fruit-7 diseased_fruit 0.94')


def test_fruit_visualization_draws_boxes_labels_and_only_diseased_aim_points(monkeypatch):
    calls = {'rectangles': [], 'labels': [], 'circles': []}
    monkeypatch.setattr('wvcsc_rgb_vision.two_stage_yolo.cv2.rectangle',
                        lambda *_args: calls['rectangles'].append(_args[1:]))
    monkeypatch.setattr('wvcsc_rgb_vision.two_stage_yolo.cv2.putText',
                        lambda *_args: calls['labels'].append(_args[1:]))
    monkeypatch.setattr('wvcsc_rgb_vision.two_stage_yolo.cv2.circle',
                        lambda *_args: calls['circles'].append(_args[1:]))
    healthy = Instance(
        'fruit-1', 'healthy_fruit', 0.9, 1, 2, 11, 12, 6, 7)
    diseased = Instance(
        'fruit-2', 'diseased_fruit', 0.8, 20, 21, 40, 41, 30, 31)

    rendered = TwoStageYolo._annotated_image(
        np.zeros((64, 64, 3), dtype=np.uint8), [healthy, diseased],
        draw_diseased_aim_point=True)

    assert rendered.shape == (64, 64, 3)
    assert [entry[0:2] for entry in calls['rectangles']] == [
        ((1, 2), (11, 12)), ((20, 21), (40, 41))]
    assert [entry[0] for entry in calls['labels']] == [
        'fruit-1 healthy_fruit 0.90',
        'fruit-2 diseased_fruit 0.80',
    ]
    assert [entry[0] for entry in calls['circles']] == [(30, 31)]


def test_selected_target_has_a_separate_visual_highlight(monkeypatch):
    rectangles = []
    circles = []
    monkeypatch.setattr(
        'wvcsc_rgb_vision.two_stage_yolo.cv2.rectangle',
        lambda *_args: rectangles.append(_args[1:]))
    monkeypatch.setattr(
        'wvcsc_rgb_vision.two_stage_yolo.cv2.putText',
        lambda *_args: None)
    monkeypatch.setattr(
        'wvcsc_rgb_vision.two_stage_yolo.cv2.circle',
        lambda *_args: circles.append(_args[1:]))
    selected = Instance(
        'fruit-2', 'diseased_fruit', 0.8, 20, 21, 40, 41, 30, 31)

    TwoStageYolo._annotated_image(
        np.zeros((64, 64, 3), dtype=np.uint8), [selected],
        draw_diseased_aim_point=True, selected_target_id='fruit-2')

    assert rectangles[0][-1] == 4
    assert circles[0][1] == 5


def test_visualization_images_keep_the_camera_header():
    class Publisher:
        messages = []

        def publish(self, message):
            self.messages.append(message)

    class Bridge:
        def cv2_to_imgmsg(self, image, encoding):
            assert encoding == 'bgr8'
            return SimpleNamespace(image=image, header=None)

    node = object.__new__(TwoStageYolo)
    node._bridge = Bridge()
    header = SimpleNamespace(frame_id='camera_color_optical_frame')
    publisher = Publisher()
    node._publish_visualization(
        publisher, SimpleNamespace(header=header), np.zeros((8, 8, 3), dtype=np.uint8))

    assert publisher.messages[0].header is header


def test_stage_visualizations_follow_the_inference_mode():
    class Publisher:
        def __init__(self):
            self.messages = []

        def publish(self, message):
            self.messages.append(message)

    tree = Instance('', 'tree', 0.9, 1, 2, 31, 42, 16, 22)
    fruit = Instance('fruit-1', 'diseased_fruit', 0.8, 10, 12, 20, 24, 15, 18)
    node = object.__new__(TwoStageYolo)
    node._tree_id = 'tree_01'
    node._bridge = SimpleNamespace(
        imgmsg_to_cv2=lambda *_args, **_kwargs: np.zeros(
            (64, 64, 3), dtype=np.uint8))
    node._tree_pub = Publisher()
    node._fruit_pub = Publisher()
    node._best_tree = lambda _image: tree
    node._fruit_instances = lambda _image, _tree: [fruit]
    node._assign_track_ids = lambda instances: instances
    node._array = lambda _message, instances: list(instances)
    node.get_parameter = lambda _name: SimpleNamespace(value=True)
    published = []
    node._publish_tree_visualization = lambda *_args: published.append('tree')
    node._publish_fruit_visualization = lambda *_args: published.append('fruit')
    node._publish_perception_debug = lambda *_args: None

    node._inference_mode = 'tree'
    node._on_image(SimpleNamespace())
    assert published == ['tree']
    assert len(node._fruit_pub.messages) == 0

    node._inference_mode = 'fruits'
    node._on_image(SimpleNamespace())
    assert published == ['tree', 'tree', 'fruit']
    assert len(node._fruit_pub.messages) == 1

    node._best_tree = lambda _image: None
    node._on_image(SimpleNamespace())
    assert published == ['tree', 'tree', 'fruit', 'tree']
    assert len(node._fruit_pub.messages) == 2
