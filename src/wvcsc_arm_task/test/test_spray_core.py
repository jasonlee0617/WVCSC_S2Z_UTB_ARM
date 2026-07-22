import pytest

from wvcsc_arm_task.core import SprayInterlock


@pytest.mark.parametrize('mission,tree,duration,mode', [
    ('', 'tree', 1.0, 'continuous'),
    ('mission', '', 1.0, 'continuous'),
    ('mission', 'tree', float('nan'), 'continuous'),
    ('mission', 'tree', 0.1, 'continuous'),
    ('mission', 'tree', 1.0, 'pulse'),
])
def test_invalid_goal_is_rejected(mission, tree, duration, mode):
    interlock = SprayInterlock()
    assert interlock.validate(mission, tree, duration, mode)


def test_single_goal_and_motion_lock_interlock():
    interlock = SprayInterlock()
    assert not interlock.validate('mission', 'tree', 1.0, 'continuous')
    assert interlock.claim()
    assert not interlock.claim()
    interlock.release()
    interlock.set_motion_locked(True)
    assert not interlock.claim()
    interlock.set_motion_locked(False)
    assert interlock.claim()
