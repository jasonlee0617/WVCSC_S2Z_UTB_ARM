from wvcsc_arm_task.target_flow import FruitTarget, associate_known_targets


def _target(target_id, u, v, confidence=0.8):
    return FruitTarget(target_id, confidence, u, v, 20.0, 20.0)


def _same_target(left, right):
    return left.iou(right) >= 0.30 or left.distance_to(right) <= 18.0


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
