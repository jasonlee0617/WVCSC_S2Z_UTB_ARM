from wvcsc_safety.core import (
    Freshness,
    base_is_stopped,
    latch_can_clear,
    velocity_allowed,
)


def test_velocity_requires_every_gate():
    fresh = Freshness(command=9.9, odom=9.8, scan=9.5, imu=9.5)
    kwargs = dict(
        autonomy_enabled=True, stop_latched=False, emergency_stop=False,
        freshness=fresh, now=10.0, command_timeout=0.3,
        odom_timeout=0.5, scan_timeout=1.0, imu_timeout=1.0)
    assert velocity_allowed(**kwargs)
    assert not velocity_allowed(**{**kwargs, 'autonomy_enabled': False})
    assert not velocity_allowed(**{**kwargs, 'stop_latched': True})
    assert not velocity_allowed(**{**kwargs, 'emergency_stop': True})
    assert not velocity_allowed(**{
        **kwargs, 'freshness': Freshness(9.0, 9.8, 9.5, 9.5)})


def test_base_stop_thresholds_are_inclusive_and_finite():
    assert base_is_stopped(0.03, -0.03, 0.03, 0.03)
    assert not base_is_stopped(0.031, 0.0, 0.03, 0.03)
    assert not base_is_stopped(float('nan'), 0.0, 0.03, 0.03)


def test_latch_only_clears_after_home_and_without_emergency():
    assert latch_can_clear(
        emergency_stop=False, recovery_active=False, arm_state='HOME_LOCKED')
    assert not latch_can_clear(
        emergency_stop=True, recovery_active=False, arm_state='HOME_LOCKED')
    assert not latch_can_clear(
        emergency_stop=False, recovery_active=True, arm_state='HOME_LOCKED')
    assert not latch_can_clear(
        emergency_stop=False, recovery_active=False, arm_state='STOPPED_LOCKED')
