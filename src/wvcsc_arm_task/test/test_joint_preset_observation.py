import math
import threading
from types import SimpleNamespace

import pytest

from wvcsc_arm_task.observation import ObservationCandidate
from wvcsc_arm_task.spray_task import DEFAULT_JOINT_PRESETS_DEG, SprayTask


class _PresetParameterHarness:
    _joint_preset_parameters = SprayTask._joint_preset_parameters

    def __init__(self, values):
        self._values = values

    def get_parameter(self, name):
        return SimpleNamespace(value=self._values[name])


def _parameter_values(**overrides):
    values = {
        'observation_mode': 'joint_presets',
        'joint_preset_center_deg': list(DEFAULT_JOINT_PRESETS_DEG['center']),
        'joint_preset_fan_left_deg': list(
            DEFAULT_JOINT_PRESETS_DEG['fan_left']),
        'joint_preset_fan_right_deg': list(
            DEFAULT_JOINT_PRESETS_DEG['fan_right']),
    }
    values.update(overrides)
    return values


def test_joint_presets_convert_configured_degrees_to_radians_exactly():
    mode, presets = _PresetParameterHarness(
        _parameter_values())._joint_preset_parameters()

    assert mode == 'joint_presets'
    assert [name for name, _joints in presets] == [
        'center', 'fan_left', 'fan_right']
    assert presets[0][1] == pytest.approx(tuple(
        math.radians(value) for value in DEFAULT_JOINT_PRESETS_DEG['center']))
    assert presets[1][1] == pytest.approx(tuple(
        math.radians(value) for value in DEFAULT_JOINT_PRESETS_DEG['fan_left']))
    assert presets[2][1] == pytest.approx(tuple(
        math.radians(value) for value in DEFAULT_JOINT_PRESETS_DEG['fan_right']))


@pytest.mark.parametrize('parameter, value', [
    ('joint_preset_center_deg', [0.0] * 5),
    ('joint_preset_fan_left_deg', [0.0, 0.0, 0.0, 0.0, 0.0, math.nan]),
])
def test_joint_preset_parameters_reject_malformed_joint_lists(parameter, value):
    with pytest.raises(ValueError, match='six finite degrees'):
        _PresetParameterHarness(_parameter_values(
            **{parameter: value}))._joint_preset_parameters()


def test_invalid_observation_mode_is_rejected():
    with pytest.raises(ValueError, match='observation_mode'):
        _PresetParameterHarness(_parameter_values(
            observation_mode='forward_kinematics'))._joint_preset_parameters()


class _Logger:
    def __init__(self):
        self.messages = []

    def error(self, message):
        self.messages.append(message)

    def warn(self, message):
        self.messages.append(message)

    def info(self, message):
        self.messages.append(message)


class _SideGateHarness:
    _move_to_observation = SprayTask._move_to_observation
    _hint_available = staticmethod(SprayTask._hint_available)

    def __init__(self):
        self._observation_mode = 'joint_presets'
        self._base_frame = 'alicia_base_link'
        self._camera_frame = 'camera_color_optical_frame'
        self._observation_failure_reason = ''
        self._tf_buffer = self
        self.lookup_calls = []
        self.logger = _Logger()
        self.motion_attempted = False

    def lookup_transform(self, target, source, _time):
        self.lookup_calls.append((target, source))
        return SimpleNamespace(transform=SimpleNamespace(
            translation=SimpleNamespace(x=0.0, y=0.0, z=0.0),
            rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)))

    def get_logger(self):
        return self.logger

    def _publish_observation_debug(self, *_args, **_kwargs):
        pass

    def _prepare_joint_preset_observation_candidates(self):
        self.motion_attempted = True
        return True

    def _move_to_next_observation(self):
        self.motion_attempted = True
        return True


def test_joint_preset_rejects_right_or_center_tree_before_any_arm_motion():
    task = _SideGateHarness()
    hint = SimpleNamespace(
        header=SimpleNamespace(frame_id='alicia_base_link'),
        point=SimpleNamespace(x=0.2, y=0.0, z=0.0))

    assert not task._move_to_observation(hint)
    assert task.motion_attempted is False
    assert task._observation_failure_reason.startswith(
        'joint_preset_tree_side_unsupported')
    assert task.lookup_calls == [('alicia_base_link', 'alicia_base_link')]


def _preset(name, joints):
    return ObservationCandidate(
        candidate_id=f'joint_preset_{name}',
        distance_m=1.0,
        camera_height_m=0.0,
        azimuth_deg=0.0,
        camera_position=(0.0, 0.0, 0.0),
        camera_quat=(0.0, 0.0, 0.0, 1.0),
        tool_position=(0.0, 0.0, 0.0),
        tool_quat=(0.0, 0.0, 0.0, 1.0),
        visible=True,
        visible_margin_px=math.inf,
        observation_mode='joint_presets',
        joint_positions=tuple(joints),
    )


class _PresetMotionHarness:
    _move_to_next_observation = SprayTask._move_to_next_observation
    _return_to_observation = SprayTask._return_to_observation

    def __init__(self):
        self._observation_candidates = [
            _preset('center', (0.1,) * 6),
            _preset('fan_left', (0.2,) * 6),
            _preset('fan_right', (0.3,) * 6),
        ]
        self._observation_candidate_index = -1
        self._observation_failure_reason = ''
        self._observation_distance = None
        self._observation_pose = None
        self._spray_working_distance = 1.0
        self._abort = threading.Event()
        self.moves = []
        self.logger = _Logger()
        self.arm = self

    @staticmethod
    def _aborted(_cancel_requested):
        return False

    def move_joints(self, joints):
        self.moves.append(tuple(joints))
        return len(self.moves) != 1

    @staticmethod
    def _current_camera_pose():
        return ((0.1, 0.2, 0.3), (0.0, 0.0, 0.0, 1.0))

    @staticmethod
    def _publish_observation_debug(*_args, **_kwargs):
        pass

    def get_logger(self):
        return self.logger


def test_joint_presets_scan_in_fixed_order_skip_failure_and_return_selected_pose():
    task = _PresetMotionHarness()

    assert task._move_to_next_observation()
    assert task.moves == [(0.1,) * 6, (0.2,) * 6]
    assert task._observation_candidate_index == 1
    assert task._observation_distance == pytest.approx(1.0)
    assert task._return_to_observation()
    assert task.moves[-1] == (0.2,) * 6
