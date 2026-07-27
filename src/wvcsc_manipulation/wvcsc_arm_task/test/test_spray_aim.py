from types import SimpleNamespace

import pytest

from wvcsc_arm_task.spray_aim import SprayAimMixin


class _CompletedFuture:
    @staticmethod
    def done():
        return True

    @staticmethod
    def result():
        return SimpleNamespace(
            success=True,
            desired_u_px=320.0,
            desired_v_px=240.0,
            image_width=640,
            image_height=480,
            message='',
        )


class _AimClient:
    def __init__(self):
        self.requested_ranges = []

    @staticmethod
    def service_is_ready():
        return True

    def call_async(self, request):
        self.requested_ranges.append(float(request.working_range_m))
        return _CompletedFuture()


class _AimHarness:
    _request_spray_aim = SprayAimMixin._request_spray_aim

    def __init__(self, override):
        self._working_range_override = override
        self._aim_client = _AimClient()
        self._active_aim = None
        self.dynamic_calls = 0

    @staticmethod
    def get_parameter(_name):
        return SimpleNamespace(value=0.1)

    @staticmethod
    def _aborted(_cancel_requested):
        return False

    def _dynamic_nozzle_range(self):
        self.dynamic_calls += 1
        return 1.13, ''

    @staticmethod
    def get_logger():
        return SimpleNamespace(info=lambda *_args: None)


def test_manual_working_range_bypasses_dynamic_tree_geometry():
    task = _AimHarness(0.9)

    assert task._request_spray_aim(lambda: False) == (True, '')
    assert task.dynamic_calls == 0
    assert task._aim_client.requested_ranges == pytest.approx([0.9])
    assert task._active_aim[4] == pytest.approx(0.9)


def test_zero_working_range_keeps_dynamic_tree_geometry():
    task = _AimHarness(0.0)

    assert task._request_spray_aim(lambda: False) == (True, '')
    assert task.dynamic_calls == 1
    assert task._aim_client.requested_ranges == pytest.approx([1.13])
    assert task._active_aim[4] == pytest.approx(1.13)
