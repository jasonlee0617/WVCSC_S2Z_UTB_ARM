from types import SimpleNamespace

from wvcsc_mission_manager.relay_controller import RelayController


class _Future:
    def __init__(self, result):
        self._result = result

    def result(self):
        return self._result

    def add_done_callback(self, callback):
        callback(self)


class _Logger:
    def info(self, _message):
        pass

    def warning(self, _message):
        pass


class _Client:
    def __init__(self):
        self.ready = True
        self.response = SimpleNamespace(success=True, message='ok')

    def service_is_ready(self):
        return self.ready

    def call_async(self, _request):
        return _Future(self.response)


class _Publisher:
    def __init__(self):
        self.values = []

    def publish(self, message):
        self.values.append(bool(message.data))


class _Node:
    def __init__(self):
        self.client = _Client()
        self.publisher = _Publisher()

    def create_client(self, _service_type, _service_name):
        return self.client

    def create_publisher(self, _message_type, _topic, _qos):
        return self.publisher

    @staticmethod
    def get_logger():
        return _Logger()


def test_wide_status_changes_only_after_a_confirmed_relay_response():
    node = _Node()
    failures = []
    relay = RelayController(
        node, service_name='/relay/set', wide_channel=1, arm_channel=2,
        require_service=False, status_qos=10, required_failure_callback=failures.append)

    relay.command(1, True, 0.0, None, 'wide on')
    assert node.publisher.values == [False, True]
    assert relay.wide_enabled is True

    node.client.response = SimpleNamespace(success=False, message='rejected off')
    relay.command(1, False, 0.0, None, 'wide off')
    assert node.publisher.values == [False, True]
    assert relay.wide_enabled is True
    assert failures == []

    node.client.response = SimpleNamespace(success=True, message='ok')
    relay.command(1, False, 0.0, None, 'wide off')
    assert node.publisher.values == [False, True, False]
    assert relay.wide_enabled is False
