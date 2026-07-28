import pytest

from wvcsc_arm_task.core import SprayInterlock


@pytest.mark.parametrize('mission,duration,mode', [
    ('', 1.0, 'continuous'),
    ('mission', float('nan'), 'continuous'),
    ('mission', 0.1, 'continuous'),
    ('mission', 1.0, 'pulse'),
])
def test_invalid_goal_is_rejected(mission, duration, mode):
    interlock = SprayInterlock()
    assert interlock.validate(mission, duration, mode)


def test_single_goal_and_motion_lock_interlock():
    interlock = SprayInterlock()
    assert not interlock.validate('mission', 1.0, 'continuous')
    assert interlock.claim()
    assert not interlock.claim()
    interlock.release()
    interlock.set_motion_locked(True)
    assert not interlock.claim()
    interlock.set_motion_locked(False)
    assert interlock.claim()
