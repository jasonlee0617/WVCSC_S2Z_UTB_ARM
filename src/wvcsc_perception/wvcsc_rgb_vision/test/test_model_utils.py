import numpy as np
import pytest
from uuid import UUID
from std_msgs.msg import Header
from types import SimpleNamespace
from wvcsc_interfaces.msg import MissionStatus

from wvcsc_rgb_vision.model_utils import (
    CANONICAL_DISEASE_TARGET_CLASS_NAME,
    validate_yolo_model,
)
from wvcsc_rgb_vision.disease_segmenter import DiseaseSegmenter, safest_mask_point
from wvcsc_rgb_vision.disease_detector import DiseaseDetector
from wvcsc_rgb_vision import disease_backend_factory
from wvcsc_rgb_vision.perception_pipeline import (
    Instance,
    Track,
    PerceptionPipeline,
    deduplicate_instances,
    match_target_template,
    track_matches,
)
from wvcsc_rgb_vision.target_tracking import capture_target_template
from wvcsc_rgb_vision.perception_output import (
    annotated_image, instance_label, instance_to_detection)
from wvcsc_rgb_vision.perception_types import DiseaseTarget


def test_disease_backend_factory_selects_only_explicit_backends(monkeypatch):
    created = []

    def fake_backend(*args, **kwargs):
        created.append((args, kwargs))
        return 'backend'

    monkeypatch.setattr(
        disease_backend_factory, 'DiseaseSegmenter', fake_backend)
    assert disease_backend_factory.create_disease_backend(
        'segment', 'weights.pt', 2, 'disease_leaf',
        strict_model_classes=True) == 'backend'
    assert created == [(
        ('weights.pt', 2, 'disease_leaf'),
        {'strict_model_classes': True},
    )]
    with pytest.raises(ValueError, match='segment.*detect'):
        disease_backend_factory.create_disease_backend(
            'unknown', 'weights.pt', 2, 'disease_leaf')


def test_disease_model_uses_the_shared_target_contract():
    assert CANONICAL_DISEASE_TARGET_CLASS_NAME == 'diseased_target'


def test_deployment_weight_must_match_task_and_classes():
    model = type('Model', (), {
        'task': 'segment',
        'names': {0: 'diseased_fruit'},
    })()
    validate_yolo_model(model, 'segment', {0: 'diseased_fruit'})
    model.names[0] = 'wrong_class'
    with pytest.raises(ValueError, match='contract mismatch'):
        validate_yolo_model(model, 'segment', {0: 'diseased_fruit'})


def test_real_deployment_rejects_additional_model_classes():
    model = type('Model', (), {
        'task': 'segment',
        'names': {0: 'disease_leaf', 1: 'healthy_leaf'},
    })()

    validate_yolo_model(model, 'segment', {0: 'disease_leaf'})
    with pytest.raises(ValueError, match='contract mismatch'):
        validate_yolo_model(
            model, 'segment', {0: 'disease_leaf'}, exact_names=True)

    canonical_checkpoint = type('Model', (), {
        'task': 'segment',
        'names': {0: 'diseased_target'},
    })()
    with pytest.raises(ValueError, match='contract mismatch'):
        validate_yolo_model(
            canonical_checkpoint, 'segment', {0: 'disease_leaf'},
            exact_names=True)


def test_detect_experiment_accepts_a_configured_disease_class_in_a_multiclass_model():
    model = type('Model', (), {
        'task': 'detect',
        'names': {0: 'healthy_leaf', 2: 'disease_leaf'},
    })()

    validate_yolo_model(model, 'detect', {2: 'disease_leaf'})
    with pytest.raises(ValueError, match='contract mismatch'):
        validate_yolo_model(model, 'detect', {1: 'disease_leaf'})


def test_detection_message_exposes_only_shared_target_class():
    instance = Instance(
        'target-1', 'disease_leaf', 0.8,
        10.0, 20.0, 30.0, 40.0, 20.0, 30.0)
    detection = instance_to_detection(Header(), instance)
    assert detection.results[0].hypothesis.class_id == 'diseased_target'


def test_mission_status_activates_and_clears_task_vision_identity():
    class Logger:
        def __init__(self):
            self.messages = []

        def info(self, message):
            self.messages.append(message)

    node = object.__new__(PerceptionPipeline)
    node._standalone_mode = False
    node._mission_id = ''
    node._point_id = ''
    node._selected_target_id = 'stale-target'
    node._selected_target_reference = object()
    node._selected_target_template = object()
    reset_calls = []
    logger = Logger()
    node._reset_tracking = lambda: reset_calls.append(True)
    node.get_logger = lambda: logger

    PerceptionPipeline._on_status(node, SimpleNamespace(
        state=MissionStatus.ARM_SPRAYING,
        mission_id='corn_field_five_point_001', current_point_id='point_02'))
    assert (node._mission_id, node._point_id) == (
        'corn_field_five_point_001', 'point_02')
    assert reset_calls == [True]
    assert node._selected_target_id == ''
    assert node._selected_target_reference is None
    assert node._selected_target_template is None
    assert logger.messages == [
        '[YOLO][MISSION] active=True mission=corn_field_five_point_001 point=point_02']

    PerceptionPipeline._on_status(node, SimpleNamespace(
        state=MissionStatus.NAVIGATING,
        mission_id='corn_field_five_point_001', current_point_id=''))
    assert (node._mission_id, node._point_id) == ('', '')
    assert node._selected_target_id == ''
    assert node._selected_target_reference is None
    assert node._selected_target_template is None


def test_fruit_tracks_survive_a_short_detector_dropout():
    node = object.__new__(PerceptionPipeline)
    node._tracks = []
    node.get_parameter = lambda name: type('Parameter', (), {'value': {
        'track_iou_threshold': 0.30,
        'track_center_distance_px': 40.0,
        'track_max_missed_frames': 3,
    }[name]})()
    fruit = Instance('', 'diseased_fruit', 0.9, 10.0, 10.0, 30.0, 30.0, 20.0, 20.0)
    first = node._assign_track_ids([fruit])[0]
    assert str(UUID(first.target_id)) == first.target_id
    node._assign_track_ids([])
    again = node._assign_track_ids([fruit])[0]
    assert first.target_id == again.target_id


def test_locked_template_tracks_target_through_detector_dropout():
    image = np.zeros((120, 160, 3), dtype=np.uint8)
    pattern = np.array([
        [[20, 40, 180], [30, 180, 240], [10, 70, 130]],
        [[50, 220, 250], [0, 255, 255], [40, 100, 170]],
        [[10, 30, 100], [60, 150, 230], [25, 60, 140]],
    ], dtype=np.uint8)
    image[40:43, 50:53] = pattern
    target = Instance(
        'fruit-1', 'diseased_fruit', 0.80,
        50, 40, 53, 43, 51, 41)
    template = capture_target_template(
        image, target, padding_ratio=0.0, min_padding_px=0.0)
    assert template is not None

    moved = np.zeros_like(image)
    moved[72:75, 84:87] = pattern
    tracked = match_target_template(
        moved, template, target, search_radius_px=50, min_score=0.90)

    assert tracked is not None
    assert tracked.target_id == 'fruit-1'
    assert tracked.confidence == pytest.approx(0.80)
    assert tracked.aim_u == pytest.approx(85.0)
    assert tracked.aim_v == pytest.approx(73.0)


def test_locked_template_rejects_an_unrelated_search_region():
    image = np.zeros((40, 40, 3), dtype=np.uint8)
    image[10:13, 10:13] = np.array([
        [[0, 20, 100], [0, 80, 180], [10, 30, 90]],
        [[5, 120, 220], [0, 255, 255], [10, 70, 160]],
        [[20, 40, 110], [0, 100, 200], [5, 20, 80]],
    ], dtype=np.uint8)
    target = Instance(
        'fruit-1', 'diseased_fruit', 0.80,
        10, 10, 13, 13, 11, 11)
    template = capture_target_template(
        image, target, padding_ratio=0.0, min_padding_px=0.0)

    assert match_target_template(
        np.zeros_like(image), template, target,
        search_radius_px=10, min_score=0.90) is None


def test_valid_low_confidence_yolo_is_not_replaced_by_template():
    initial = np.zeros((100, 120, 3), dtype=np.uint8)
    pattern = np.arange(5 * 5 * 3, dtype=np.uint8).reshape(5, 5, 3)
    initial[20:25, 30:35] = pattern
    reference = Instance(
        'fruit-1', 'diseased_fruit', 0.80,
        30, 20, 35, 25, 32, 22)
    template = capture_target_template(
        initial, reference, padding_ratio=0.0, min_padding_px=0.0)
    moved = np.zeros_like(initial)
    moved[44:49, 52:57] = pattern
    low_confidence = Instance(
        'fruit-9', 'diseased_fruit', 0.12,
        52, 44, 57, 49, 54, 46)

    node = object.__new__(PerceptionPipeline)
    node._selected_target_id = 'fruit-1'
    node._selected_target_reference = reference
    node._selected_target_template = template
    node.get_parameter = lambda name: SimpleNamespace(value={
        'track_iou_threshold': 0.20,
        'track_center_distance_px': 50.0,
        'target_reassociation_iou_margin': 0.10,
        'target_reassociation_distance_margin_px': 8.0,
        'target_reassociation_distance_px': 50.0,
        'target_equivalent_aim_distance_px': 8.0,
        'target_lock_ema_alpha': 0.50,
        'target_template_tracking_enabled': True,
        'target_template_update_min_confidence': 0.30,
        'target_template_padding_ratio': 0.0,
        'target_template_min_padding_px': 0.0,
        'target_template_search_radius_px': 50.0,
        'target_template_min_score': 0.90,
    }[name])

    tracked, reason, event = node._resolve_or_track_selected_target(
        moved, [low_confidence])

    assert reason == 'none'
    assert event == 'target_reassociated'
    assert tracked.confidence == pytest.approx(0.12)
    assert tracked.aim_u == pytest.approx(43.0)
    assert tracked.aim_v == pytest.approx(34.0)


def test_template_only_bridges_a_frame_with_no_yolo_candidates():
    initial = np.zeros((100, 120, 3), dtype=np.uint8)
    pattern = np.arange(5 * 5 * 3, dtype=np.uint8).reshape(5, 5, 3)
    initial[20:25, 30:35] = pattern
    reference = Instance(
        'fruit-1', 'diseased_fruit', 0.80,
        30, 20, 35, 25, 32, 22)
    template = capture_target_template(
        initial, reference, padding_ratio=0.0, min_padding_px=0.0)
    moved = np.zeros_like(initial)
    moved[44:49, 52:57] = pattern
    node = object.__new__(PerceptionPipeline)
    node._selected_target_id = 'fruit-1'
    node._selected_target_reference = reference
    node._selected_target_template = template
    node.get_parameter = lambda name: SimpleNamespace(value={
        'track_iou_threshold': 0.20,
        'target_reassociation_distance_px': 50.0,
        'target_reassociation_iou_margin': 0.10,
        'target_reassociation_distance_margin_px': 8.0,
        'target_equivalent_aim_distance_px': 8.0,
        'target_lock_ema_alpha': 0.50,
        'target_template_tracking_enabled': True,
        'target_template_update_min_confidence': 0.30,
        'target_template_padding_ratio': 0.0,
        'target_template_min_padding_px': 0.0,
        'target_template_search_radius_px': 50.0,
        'target_template_min_score': 0.90,
    }[name])

    tracked, reason, event = node._resolve_or_track_selected_target(moved, [])

    assert reason == 'none'
    assert event == 'target_template_tracked'
    assert tracked.aim_u == pytest.approx(54.0)
    assert tracked.aim_v == pytest.approx(46.0)


def test_template_does_not_bypass_an_ambiguous_yolo_frame():
    initial = np.zeros((80, 120, 3), dtype=np.uint8)
    pattern = np.arange(5 * 5 * 3, dtype=np.uint8).reshape(5, 5, 3)
    initial[20:25, 30:35] = pattern
    reference = Instance(
        'fruit-1', 'diseased_fruit', 0.80,
        30, 20, 35, 25, 32, 22)
    node = object.__new__(PerceptionPipeline)
    node._selected_target_id = 'fruit-1'
    node._selected_target_reference = reference
    node._selected_target_template = capture_target_template(
        initial, reference, padding_ratio=0.0, min_padding_px=0.0)
    node.get_parameter = lambda name: SimpleNamespace(value={
        'track_iou_threshold': 0.20,
        'target_reassociation_distance_px': 50.0,
        'target_reassociation_require_unique_candidate': True,
        'target_reassociation_iou_margin': 0.10,
        'target_reassociation_distance_margin_px': 8.0,
        'target_equivalent_aim_distance_px': 8.0,
        'target_lock_ema_alpha': 0.50,
        'target_template_tracking_enabled': True,
        'target_template_update_min_confidence': 0.30,
        'target_template_padding_ratio': 0.0,
        'target_template_min_padding_px': 0.0,
        'target_template_search_radius_px': 50.0,
        'target_template_min_score': 0.50,
    }[name])
    candidates = [
        Instance('fruit-9', 'diseased_fruit', 0.8, 31, 20, 36, 25, 33, 22),
        Instance('fruit-10', 'diseased_fruit', 0.8, 60, 20, 65, 25, 62, 22),
    ]

    target, reason, event = node._resolve_or_track_selected_target(
        initial, candidates)

    assert target is None
    assert reason == 'selected_id_missing_multiple_candidates'
    assert event == 'target_invalid'


def _target_selection_node(selected_id, reference):
    node = object.__new__(PerceptionPipeline)
    node._selected_target_id = selected_id
    node._selected_target_reference = reference
    node.get_parameter = lambda name: SimpleNamespace(value={
        'track_iou_threshold': 0.30,
        'track_center_distance_px': 40.0,
        'target_reassociation_iou_margin': 0.10,
        'target_reassociation_distance_margin_px': 8.0,
        'target_reassociation_distance_px': 40.0,
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


def test_selected_target_keeps_exact_tracker_id_after_large_camera_motion():
    reference = Instance('fruit-1', 'diseased_fruit', 0.9, 10, 10, 30, 30, 20, 20)
    same_id = Instance('fruit-1', 'diseased_fruit', 0.9, 130, 10, 150, 30, 140, 20)
    node = _target_selection_node('fruit-1', reference)
    node.get_parameter = lambda name: SimpleNamespace(value={
        'track_iou_threshold': 0.30,
        'target_reassociation_distance_px': 160.0,
        'target_lock_ema_alpha': 0.35,
    }[name])

    target, reason, event = node._resolve_selected_target([same_id])

    assert target.target_id == 'fruit-1'
    assert reason == 'none'
    assert event == 'target_valid'


def test_repeated_selected_target_message_preserves_geometric_reference():
    reference = Instance(
        'fruit-9', 'diseased_fruit', 0.9, 12, 10, 32, 30, 22, 20)
    node = object.__new__(PerceptionPipeline)
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
    node._target_pub = Publisher()
    node._log_target_state = lambda *_args: None
    image = SimpleNamespace(
        header=Header(), width=640, height=480)

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


def test_simulation_recenter_allows_bounded_nearest_reassociation():
    """A 20 degree recenter may move a valid target about 185 px in C10."""
    reference = Instance('fruit-1', 'diseased_fruit', 0.9, 10, 10, 30, 30, 20, 20)
    node = _target_selection_node('fruit-1', reference)
    original_get_parameter = node.get_parameter
    node.get_parameter = lambda name: (
        SimpleNamespace(value=320.0)
        if name == 'target_reassociation_distance_px' else
        SimpleNamespace(value=True)
        if name == 'target_reassociation_allow_ambiguous_nearest' else
        original_get_parameter(name))
    candidates = [
        Instance('fruit-9', 'diseased_fruit', 0.9, 210, 10, 230, 30, 220, 20),
        Instance('fruit-10', 'diseased_fruit', 0.9, 215, 10, 235, 30, 225, 20),
    ]

    target, reason, event = node._resolve_selected_target(candidates)

    assert target is not None
    assert target.target_id == 'fruit-9'
    assert reason == 'none'
    assert event == 'target_reassociated'


def test_selected_target_rejects_id_switch_when_multiple_candidates_are_locked_out():
    reference = Instance('fruit-1', 'diseased_fruit', 0.9, 10, 10, 30, 30, 20, 20)
    node = _target_selection_node('fruit-1', reference)
    original_get_parameter = node.get_parameter
    node.get_parameter = lambda name: (
        SimpleNamespace(value=True)
        if name == 'target_reassociation_require_unique_candidate'
        else original_get_parameter(name))
    candidates = [
        Instance('fruit-9', 'diseased_fruit', 0.9, 11, 10, 31, 30, 21, 20),
        Instance('fruit-10', 'diseased_fruit', 0.9, 80, 10, 100, 30, 90, 20),
    ]

    target, reason, event = node._resolve_selected_target(candidates)

    assert target is None
    assert reason == 'selected_id_missing_multiple_candidates'
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


def test_mask_safe_point_is_inside_the_instance_polygon():
    polygon = np.array([[10, 10], [50, 10], [50, 50], [10, 50]], dtype=float)
    u, v = safest_mask_point(polygon, 64, 64)
    assert 20 <= u <= 40
    assert 20 <= v <= 40


def test_mask_safe_point_uses_center_of_deep_core_not_arbitrary_maximum():
    polygon = np.array([[10, 10], [50, 10], [50, 50], [10, 50]], dtype=float)

    u, v = safest_mask_point(polygon, 64, 64)

    assert (u, v) == pytest.approx((30.0, 30.0), abs=1.0)


def test_deduplication_keeps_highest_confidence_same_class_instance():
    best = Instance(
        '', 'diseased_fruit', 0.80, 10, 10, 30, 30, 20, 20)
    overlapping = Instance(
        '', 'diseased_fruit', 0.60, 12, 12, 32, 32, 22, 22)
    near_center = Instance(
        '', 'diseased_fruit', 0.70, 17, 17, 37, 37, 27, 27)

    assert deduplicate_instances(
        [overlapping, near_center, best]) == [best]


def test_pipeline_runs_the_segmenter_on_the_full_camera_image():
    calls = []

    class Segmenter:
        def detect(self, image, confidence):
            calls.append((image.shape, confidence))
            return [stronger]

    stronger = DiseaseTarget(
        'diseased_fruit', 0.80, 5, 5, 25, 25, 17, 19)
    node = object.__new__(PerceptionPipeline)
    node._disease_backend = Segmenter()
    node.get_parameter = lambda name: SimpleNamespace(value={
        'disease_confidence': 0.10,
    }[name])
    image = np.zeros((100, 100, 3), dtype=np.uint8)

    result = node._fruit_instances(image)

    assert calls == [((100, 100, 3), 0.10)]
    assert len(result) == 1
    assert (result[0].left, result[0].top, result[0].right, result[0].bottom) == (
        5.0, 5.0, 25.0, 25.0)
    assert (result[0].aim_u, result[0].aim_v) == (17.0, 19.0)


def test_disease_segmenter_keeps_full_image_safe_mask_point():
    class Box:
        cls = np.array([0])
        conf = np.array([0.8])
        xyxy = np.array([[5.0, 5.0, 25.0, 25.0]])

    class Model:
        def __call__(self, image, **kwargs):
            assert image.shape == (100, 100, 3)
            assert kwargs == {'verbose': False, 'conf': 0.10, 'iou': 0.45}
            return [SimpleNamespace(
                names={0: 'disease_leaf'},
                boxes=[Box()],
                masks=SimpleNamespace(xy=[np.array([
                    [5, 5], [25, 5], [25, 25], [5, 25]], dtype=float)]))]

    segmenter = object.__new__(DiseaseSegmenter)
    segmenter._model = Model()
    segmenter._target_class_id = 0
    segmenter._model_target_class_name = 'disease_leaf'

    result = segmenter.detect(np.zeros((100, 100, 3), dtype=np.uint8), 0.10)

    assert len(result) == 1
    assert result[0].class_name == 'diseased_target'
    assert (result[0].left, result[0].top, result[0].right, result[0].bottom) == (
        5.0, 5.0, 25.0, 25.0)
    assert 5.0 <= result[0].control_u <= 25.0
    assert 5.0 <= result[0].control_v <= 25.0


def test_disease_detector_returns_only_full_image_configured_boxes():
    class Box:
        def __init__(self, class_id, confidence, xyxy):
            self.cls = np.array([class_id])
            self.conf = np.array([confidence])
            self.xyxy = np.array([xyxy])

    class Model:
        def __call__(self, image, **kwargs):
            assert image.shape == (100, 100, 3)
            assert kwargs == {'verbose': False, 'conf': 0.25}
            return [SimpleNamespace(boxes=[
                Box(1, 0.9, [5.0, 7.0, 25.0, 27.0]),
                Box(0, 0.8, [1.0, 2.0, 3.0, 4.0]),
            ])]

    detector = object.__new__(DiseaseDetector)
    detector._model = Model()
    detector._target_class_id = 1
    detector._model_target_class_name = 'disease_leaf'

    result = detector.detect(np.zeros((100, 100, 3), dtype=np.uint8), 0.25)

    assert result == [DiseaseTarget(
        'diseased_target', 0.9, 5.0, 7.0, 25.0, 27.0)]
    assert result[0].control_u is None
    assert result[0].control_v is None


def test_detect_backend_uses_the_full_image_box_center():
    class Detector:
        def detect(self, image, confidence):
            assert image.shape == (100, 100, 3)
            assert confidence == 0.25
            return [DiseaseTarget(
                'diseased_target', 0.9, 5.0, 7.0, 25.0, 27.0)]

    node = object.__new__(PerceptionPipeline)
    node._disease_backend = Detector()
    node.get_parameter = lambda name: SimpleNamespace(value={
        'disease_confidence': 0.25,
    }[name])

    result = node._fruit_instances(np.zeros((100, 100, 3), dtype=np.uint8))

    assert len(result) == 1
    assert (result[0].left, result[0].top, result[0].right, result[0].bottom) == (
        5.0, 7.0, 25.0, 27.0)
    assert (result[0].aim_u, result[0].aim_v) == (15.0, 17.0)
    assert (result[0].aim_u, result[0].aim_v) == (
        result[0].center_u, result[0].center_v)


def test_detect_box_center_is_published_unchanged_in_target2d():
    class Publisher:
        def __init__(self):
            self.messages = []

        def publish(self, message):
            self.messages.append(message)

    detected = Instance(
        'target-1', 'diseased_target', 0.9,
        15.0, 27.0, 35.0, 47.0, 25.0, 37.0)
    node = _target_selection_node('target-1', detected)
    node._mission_id = 'mission-1'
    node._target_pub = Publisher()
    node._log_target_state = lambda *_args: None

    node._publish_selected_target(
        SimpleNamespace(header=Header(), width=100, height=100), [detected])

    message = node._target_pub.messages[-1]
    assert (message.center_u, message.center_v) == (25.0, 37.0)
    assert (message.center_u, message.center_v) == (
        (detected.left + detected.right) / 2.0,
        (detected.top + detected.bottom) / 2.0)


def test_visualization_labels_include_id_class_and_confidence():
    instance = Instance(
        'target-7', 'diseased_target', 0.937,
        10.4, 20.6, 30.4, 40.6, 20.0, 30.0)
    assert instance_label(instance) == (
        'target-7 diseased_target 0.94')


def test_fruit_visualization_draws_diseased_box_label_and_aim_point(monkeypatch):
    calls = {'rectangles': [], 'labels': [], 'circles': []}
    monkeypatch.setattr('wvcsc_rgb_vision.perception_output.cv2.rectangle',
                        lambda *_args: calls['rectangles'].append(_args[1:]))
    monkeypatch.setattr('wvcsc_rgb_vision.perception_output.cv2.putText',
                        lambda *_args: calls['labels'].append(_args[1:]))
    monkeypatch.setattr('wvcsc_rgb_vision.perception_output.cv2.circle',
                        lambda *_args: calls['circles'].append(_args[1:]))
    diseased = Instance(
        'target-2', 'diseased_target', 0.8, 20, 21, 40, 41, 30, 31)

    rendered = annotated_image(
        np.zeros((64, 64, 3), dtype=np.uint8), [diseased],
        draw_diseased_aim_point=True)

    assert rendered.shape == (64, 64, 3)
    assert [entry[0:2] for entry in calls['rectangles']] == [
        ((20, 21), (40, 41))]
    assert [entry[0] for entry in calls['labels']] == [
        'target-2 diseased_target 0.80',
    ]
    assert [entry[0] for entry in calls['circles']] == [(30, 31)]


def test_selected_target_has_a_separate_visual_highlight(monkeypatch):
    rectangles = []
    circles = []
    monkeypatch.setattr(
        'wvcsc_rgb_vision.perception_output.cv2.rectangle',
        lambda *_args: rectangles.append(_args[1:]))
    monkeypatch.setattr(
        'wvcsc_rgb_vision.perception_output.cv2.putText',
        lambda *_args: None)
    monkeypatch.setattr(
        'wvcsc_rgb_vision.perception_output.cv2.circle',
        lambda *_args: circles.append(_args[1:]))
    selected = Instance(
        'target-2', 'diseased_target', 0.8, 20, 21, 40, 41, 30, 31)

    annotated_image(
        np.zeros((64, 64, 3), dtype=np.uint8), [selected],
        draw_diseased_aim_point=True, selected_target_id='target-2')

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

    node = object.__new__(PerceptionPipeline)
    node._bridge = Bridge()
    header = SimpleNamespace(frame_id='camera_color_optical_frame')
    publisher = Publisher()
    node._publish_visualization(
        publisher, SimpleNamespace(header=header), np.zeros((8, 8, 3), dtype=np.uint8))

    assert publisher.messages[0].header is header


def test_disease_visualization_follows_the_inference_mode():
    class Publisher:
        def __init__(self):
            self.messages = []

        def publish(self, message):
            self.messages.append(message)

    fruit = Instance('target-1', 'diseased_target', 0.8, 10, 12, 20, 24, 15, 18)
    node = object.__new__(PerceptionPipeline)
    node._mission_id = 'mission-1'
    node._bridge = SimpleNamespace(
        imgmsg_to_cv2=lambda *_args, **_kwargs: np.zeros(
            (64, 64, 3), dtype=np.uint8))
    node._fruit_pub = Publisher()
    node._fruit_instances = lambda _image: [fruit]
    node._assign_track_ids = lambda instances: instances
    node.get_parameter = lambda _name: SimpleNamespace(value=True)
    published = []
    node._publish_fruit_visualization = lambda *_args: published.append('fruit')

    node._inference_mode = 'idle'
    node._on_image(SimpleNamespace(header=Header()))
    assert published == []
    assert len(node._fruit_pub.messages) == 0

    node._inference_mode = 'disease'
    node._on_image(SimpleNamespace(header=Header()))
    assert published == ['fruit']
    assert len(node._fruit_pub.messages) == 1


def test_real_disease_target_limit_publishes_the_two_highest_confidences():
    node = object.__new__(PerceptionPipeline)
    node._max_diseased_targets = 2
    low = Instance('target-1', 'diseased_target', 0.30, 0, 0, 10, 10, 5, 5)
    medium = Instance('target-2', 'diseased_target', 0.70, 20, 0, 30, 10, 25, 5)
    high = Instance('target-3', 'diseased_target', 0.90, 40, 0, 50, 10, 45, 5)

    selected = node._limit_diseased_targets([low, medium, high])

    assert [target.target_id for target in selected] == ['target-3', 'target-2']
