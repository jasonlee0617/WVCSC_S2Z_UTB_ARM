import pytest
from fastapi import HTTPException

from wvcsc_web_ui.state import SnapshotStore
from wvcsc_web_ui.web_server import execute_command


class _Bridge:
    def __init__(self, success=True):
        self.store = SnapshotStore()
        self.success = success
        self.commands = []

    def call_command(self, command):
        self.commands.append(command)
        return {'success': self.success, 'message': f'{command} result'}


def test_allowed_command_is_forwarded():
    bridge = _Bridge()
    response = execute_command(bridge, 'start')
    assert response == {'success': True, 'message': 'start result'}
    assert bridge.commands == ['start']


def test_failed_and_unknown_commands_are_not_reported_as_success():
    bridge = _Bridge(success=False)
    with pytest.raises(HTTPException) as failed:
        execute_command(bridge, 'cancel')
    with pytest.raises(HTTPException) as unknown:
        execute_command(bridge, 'drive_forward')
    assert failed.value.status_code == 409
    assert unknown.value.status_code == 404
