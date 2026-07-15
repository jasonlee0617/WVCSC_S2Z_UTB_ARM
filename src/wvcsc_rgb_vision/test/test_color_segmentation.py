import cv2
import numpy as np

from wvcsc_rgb_vision.color_segmentation import (
    Candidate,
    safest_mask_point,
    select_best,
)


def test_safest_point_lies_inside_mask_and_away_from_boundary():
    mask = np.zeros((80, 100), dtype=np.uint8)
    cv2.rectangle(mask, (20, 10), (80, 70), 255, -1)
    u, v = safest_mask_point(mask)
    assert mask[int(v), int(u)] == 255
    assert 45 <= u <= 55
    assert 35 <= v <= 45


def test_select_best_prefers_confidence_then_area():
    low = Candidate(0, 0, 1, 1, 500, 0.0, 0.0, 0.80)
    high = Candidate(0, 0, 1, 1, 100, 0.0, 0.0, 0.90)
    assert select_best([low, high]) is high
    assert select_best([]) is None
