import pytest

from wvcsc_arm_task.target_flow import (
    FruitTarget, associate_known_targets, target_on_tree_plane)
from wvcsc_arm_task.target_ledger import FruitTarget as LedgerFruitTarget


def _target(target_id, u, v, confidence=0.8):
    return FruitTarget(target_id, confidence, u, v, 20.0, 20.0)


def _same_target(left, right):
    return left.iou(right) >= 0.30 or left.distance_to(right) <= 18.0


def test_target_flow_keeps_the_legacy_pure_target_import_path():
    assert FruitTarget is LedgerFruitTarget


def test_cross_view_association_preserves_a_logical_target_after_recenter():
    logical = _target('fruit-logical', 460.0, 360.0)
    observed = _target('fruit-new-tracker-id', 650.0, 362.0)

    associations = associate_known_targets(
        [logical], [observed], _same_target, 320.0)

    assert associations == [(logical, observed, True)]


def test_cross_view_association_is_one_to_one_and_respects_its_gate():
    logical = [_target('fruit-1', 400.0, 360.0),
               _target('fruit-2', 640.0, 360.0)]
    observations = [_target('new-1', 430.0, 360.0),
                    _target('new-2', 810.0, 360.0)]

    associations = associate_known_targets(
        logical, observations, _same_target, 180.0)

    assert [(old.target_id, new.target_id, forced)
            for old, new, forced in associations] == [
                ('fruit-1', 'new-1', True),
                ('fruit-2', 'new-2', True),
            ]
    assert associate_known_targets(
        logical, [_target('far', 1000.0, 360.0)], _same_target, 180.0) == []


def test_tree_plane_anchor_is_stable_when_camera_changes_viewpoint():
    camera = (500.0, 500.0, 320.0, 240.0, 640, 480)
    tree = (0.0, 1.5, 0.0)
    look_at_tree = (-2.0 ** -0.5, 0.0, 0.0, 2.0 ** -0.5)
    left = target_on_tree_plane(
        _target('left', 386.6666667, 240.0),
        ((-0.2, 0.0, 0.3), look_at_tree),
        camera, tree, 1)
    right = target_on_tree_plane(
        _target('right', 253.3333333, 240.0),
        ((0.2, 0.0, 0.3), look_at_tree),
        camera, tree, 2)

    assert left.tree_plane_distance_to(right) == pytest.approx(0.0, abs=1e-6)
    assert (left.observation_index, right.observation_index) == (1, 2)


def test_spatially_anchored_targets_are_not_rescued_by_wide_pixel_fallback():
    known = FruitTarget('treated', 0.9, 100.0, 100.0, 20.0, 20.0,
                        0.0, 1.0, 0)
    other = FruitTarget('other', 0.9, 260.0, 100.0, 20.0, 20.0,
                        0.30, 1.0, 1)

    assert associate_known_targets(
        [known], [other], lambda left, right: False, 320.0) == []
