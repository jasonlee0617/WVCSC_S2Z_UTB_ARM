import importlib.util
from pathlib import Path

import pytest


SCRIPT = Path(__file__).resolve().parents[1] / 'scripts' / 'capture_site_pose.py'
SPEC = importlib.util.spec_from_file_location('capture_site_pose', SCRIPT)
capture_site_pose = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(capture_site_pose)


class _NoMotionClient:
    def __init__(self, available=True):
        self.available = available
        self.calls = 0

    def service_is_ready(self):
        return self.available

    def call_async(self, _request):
        self.calls += 1


class _TransientTfBuffer:
    def __init__(self):
        self.calls = 0

    def lookup_transform(self, _target, _source, _time):
        self.calls += 1
        if self.calls == 1:
            raise capture_site_pose.TransformException(
                'Lookup would require extrapolation into the past')
        return object()


def _node():
    node = object.__new__(capture_site_pose.SitePoseCapture)
    node._imu = 9.9
    node._odom = (9.9, 0.01, 0.01)
    node._amcl = (9.9, 0.04, 0.04)
    node._stable_since = 8.0
    node._next_no_motion_update = 0.0
    node._no_motion_service_state = 'available'
    node._no_motion_client = _NoMotionClient()
    return node


def test_capture_inputs_require_fresh_imu_odom_amcl_and_stop():
    node = _node()
    assert node._input_issues(10.0) == []

    node._imu = None
    node._amcl = (7.9, 0.04, 0.04)
    issues = node._input_issues(10.0)
    assert 'AHRS /imu has not published' in issues
    assert any('AMCL /amcl_pose is stale' in issue for issue in issues)


def test_amcl_one_second_publish_jitter_is_allowed():
    node = _node()
    node._amcl = (8.5, 0.04, 0.04)
    assert not any('AMCL /amcl_pose is stale' in issue
                   for issue in node._input_issues(10.0))

    node._amcl = (7.9, 0.04, 0.04)
    assert any('AMCL /amcl_pose is stale' in issue
               for issue in node._input_issues(10.0))


def test_capture_reports_missing_no_motion_service_and_throttles_requests():
    node = _node()
    node._no_motion_client = _NoMotionClient(available=False)
    node._no_motion_service_state = 'not checked'
    node._request_no_motion_update(10.0)
    assert node._no_motion_service_state == 'unavailable'
    assert any('request_nomotion_update service is unavailable' in issue
               for issue in node._input_issues(10.0))

    node._no_motion_client = _NoMotionClient()
    node._next_no_motion_update = 0.0
    node._request_no_motion_update(10.0)
    node._request_no_motion_update(10.49)
    node._request_no_motion_update(10.50)
    assert node._no_motion_client.calls == 2
    assert node._no_motion_service_state == 'available'


def test_capture_quality_keeps_the_amcl_covariance_gate():
    quality = {
        'position_spread_m': 0.01,
        'yaw_spread_rad': 0.01,
        'max_position_stddev_m': 1.001,
        'max_yaw_stddev_rad': 0.04,
    }
    with pytest.raises(RuntimeError, match='1.001 m exceeds 1.00 m'):
        capture_site_pose.SitePoseCapture._validate_quality(quality)

    quality['max_position_stddev_m'] = 0.04
    capture_site_pose.SitePoseCapture._validate_quality(quality)


def test_relaxed_capture_reports_quality_issues_without_rejecting():
    quality = {
        'position_spread_m': 1.1,
        'yaw_spread_rad': 0.01,
        'max_position_stddev_m': 0.04,
        'max_yaw_stddev_rad': 0.04,
    }
    issues = capture_site_pose.SitePoseCapture._quality_issues(quality)
    assert issues
    capture_site_pose.SitePoseCapture._validate_quality({
        **quality, 'position_spread_m': 0.01})


def test_transient_tf_extrapolation_is_retryable():
    node = _node()
    node._tf_buffer = _TransientTfBuffer()

    transform, error = node._lookup_latest_transform()
    assert transform is None
    assert 'extrapolation into the past' in str(error)

    transform, error = node._lookup_latest_transform()
    assert transform is not None
    assert error is None
