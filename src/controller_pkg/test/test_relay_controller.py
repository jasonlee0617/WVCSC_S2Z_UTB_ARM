from types import SimpleNamespace

from controller_pkg.relay_controller import RelayController


class _Timer:
    def __init__(self, callback):
        self.callback = callback
        self.cancelled = False

    def cancel(self):
        self.cancelled = True


class _Relay:
    def __init__(self):
        self.calls = []

    def set_channel(self, channel, enabled):
        self.calls.append((channel, enabled))
        return True


class _Logger:
    def info(self, _message):
        pass

    def error(self, _message):
        pass


class _Harness:
    _write_relay = RelayController._write_relay
    _cancel_off_timer = RelayController._cancel_off_timer
    _schedule_auto_off = RelayController._schedule_auto_off
    _set_relay_callback = RelayController._set_relay_callback

    def __init__(self):
        self._relay = _Relay()
        self._active_channels = set()
        self._off_timers = {}

    def create_timer(self, _duration, callback):
        return _Timer(callback)

    def destroy_timer(self, _timer):
        pass

    def get_logger(self):
        return _Logger()


def _request(channel, enabled, duration):
    return SimpleNamespace(
        channel=channel, enabled=enabled, duration=duration)


def _response():
    return SimpleNamespace(success=False, message='')


def test_duration_auto_off_and_explicit_off_cancel_the_old_timer():
    controller = _Harness()
    response = controller._set_relay_callback(
        _request(2, True, 0.5), _response())
    assert response.success
    timer = controller._off_timers[2]

    timer.callback()
    assert controller._relay.calls == [(2, True), (2, False)]
    assert 2 not in controller._active_channels

    controller._set_relay_callback(_request(2, True, 1.0), _response())
    timer = controller._off_timers[2]
    response = controller._set_relay_callback(
        _request(2, False, 0.0), _response())
    assert response.success
    assert timer.cancelled
    assert 2 not in controller._off_timers
