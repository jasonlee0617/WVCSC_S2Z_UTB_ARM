from types import SimpleNamespace
from pathlib import Path

import pytest

from wvcsc_simulation import sim_relay


def _relay_harness():
    relay = object.__new__(sim_relay.SimRelay)
    relay._states = {1: False, 2: False}
    relay._deadlines = {1: None, 2: None}
    relay.published = []
    relay._publish = lambda channel: relay.published.append(
        (channel, relay._states[channel]))
    return relay


def _request(channel, enabled, duration):
    return SimpleNamespace(channel=channel, enabled=enabled, duration=duration)


def _response():
    return SimpleNamespace(success=None, message='')


def test_zero_duration_latches_until_an_explicit_off(monkeypatch):
    relay = _relay_harness()
    monkeypatch.setattr(sim_relay.time, 'monotonic', lambda: 10.0)

    response = sim_relay.SimRelay._set_relay(
        relay, _request(1, True, 0.0), _response())

    assert response.success is True
    assert relay._states[1] is True
    assert relay._deadlines[1] is None
    sim_relay.SimRelay._expire_pulses(relay)
    assert relay._states[1] is True

    sim_relay.SimRelay._set_relay(relay, _request(1, False, 0.0), _response())
    assert relay._states[1] is False
    assert relay._deadlines[1] is None


def test_positive_duration_auto_turns_off(monkeypatch):
    relay = _relay_harness()
    now = {'value': 5.0}
    monkeypatch.setattr(sim_relay.time, 'monotonic', lambda: now['value'])

    sim_relay.SimRelay._set_relay(relay, _request(2, True, 0.25), _response())
    assert relay._states[2] is True
    assert relay._deadlines[2] == pytest.approx(5.25)

    now['value'] = 5.24
    sim_relay.SimRelay._expire_pulses(relay)
    assert relay._states[2] is True
    now['value'] = 5.25
    sim_relay.SimRelay._expire_pulses(relay)
    assert relay._states[2] is False
    assert relay._deadlines[2] is None


@pytest.mark.parametrize('channel,duration', [(3, 0.0), (1, -0.1), (2, float('nan'))])
def test_invalid_requests_are_rejected(channel, duration):
    relay = _relay_harness()

    response = sim_relay.SimRelay._set_relay(
        relay, _request(channel, True, duration), _response())

    assert response.success is False
    assert relay._states == {1: False, 2: False}


def test_motion_lock_forces_both_channels_off():
    relay = _relay_harness()
    relay._states = {1: True, 2: True}
    relay._deadlines = {1: None, 2: 99.0}

    sim_relay.SimRelay._on_motion_locked(relay, SimpleNamespace(data=True))

    assert relay._states == {1: False, 2: False}
    assert relay._deadlines == {1: None, 2: None}


def test_installed_script_enters_its_ros_spin_loop_when_executed_directly():
    source = Path(sim_relay.__file__).read_text(encoding='utf-8')

    assert "if __name__ == '__main__':" in source
    assert '    main()' in source
