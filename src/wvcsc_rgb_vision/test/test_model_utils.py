import numpy as np
import pytest

from wvcsc_rgb_vision.model_utils import (
    FRUIT_CLASS_NAMES,
    TREE_CLASS_NAMES,
    canonical_class_name,
    validate_yolo_model,
)
from wvcsc_rgb_vision.two_stage_yolo import TwoStageYolo, expanded_roi, safest_mask_point


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


def test_assumed_tree_uses_the_complete_camera_frame():
    node = object.__new__(TwoStageYolo)
    node._tree_model = None
    tree = node._best_tree(np.zeros((720, 1280, 3), dtype=np.uint8))
    assert (tree.class_name, tree.confidence) == ('tree', 1.0)
    assert (tree.left, tree.top, tree.right, tree.bottom) == (0.0, 0.0, 1280.0, 720.0)


def test_roi_expansion_clips_to_image_bounds():
    assert expanded_roi(5, 5, 30, 30, 100, 80, 0.10) == (2, 2, 33, 33)


def test_mask_safe_point_is_inside_the_instance_polygon():
    polygon = np.array([[10, 10], [50, 10], [50, 50], [10, 50]], dtype=float)
    u, v = safest_mask_point(polygon, 64, 64)
    assert 20 <= u <= 40
    assert 20 <= v <= 40
