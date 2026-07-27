import importlib.util
from pathlib import Path

import pytest


SCRIPT = Path(__file__).parents[1] / 'scripts' / 'arm_spray_test_qt.py'
SPEC = importlib.util.spec_from_file_location('arm_spray_test_qt', SCRIPT)
arm_spray_test_qt = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(arm_spray_test_qt)


def test_single_target_goal_uses_the_selected_arm_frame_values_and_mode():
    goal = arm_spray_test_qt.build_spray_goal(
        'mission_1', 'corn_01', 'alicia_base_link',
        0.0, 1.3, 0.0, 3.0, 'ik', 0.9)
    assert goal.mission_id == 'mission_1'
    assert goal.tree_id == 'corn_01'
    assert goal.tree_hint.header.frame_id == 'alicia_base_link'
    assert (goal.tree_hint.point.x, goal.tree_hint.point.y,
            goal.tree_hint.point.z, goal.spray_duration) == (0.0, 1.3, 0.0, 3.0)
    assert goal.observation_mode == 'ik'
    assert goal.working_range_m == pytest.approx(0.9)


def test_side_distance_coordinates_encode_left_and_right():
    assert arm_spray_test_qt.side_distance_coordinates('left', 1.2) == (
        0.0, 1.2, 0.0)
    assert arm_spray_test_qt.side_distance_coordinates('right', 1.2) == (
        0.0, -1.2, 0.0)


def test_ik_uses_base_distance_but_joint_preset_uses_hidden_side_hint():
    assert arm_spray_test_qt.arm_test_coordinates(
        'ik', 'left', 1.3, 1.0) == (0.0, 1.3, 0.0)
    assert arm_spray_test_qt.arm_test_coordinates(
        'ik', 'right', 0.8, 1.0) == (0.0, -0.8, 0.0)
    assert arm_spray_test_qt.arm_test_coordinates(
        'joint_presets', 'left', 1.5, 1.0) == (0.0, 1.0, 0.0)
    assert arm_spray_test_qt.arm_test_coordinates(
        'joint_presets', 'right', 0.8, 1.0) == (0.0, -1.0, 0.0)


def test_qt_source_has_no_manual_xyz_fields_and_exposes_manual_working_range():
    source = SCRIPT.read_text(encoding='utf-8')
    for removed in ('self.x_spin', 'self.y_spin', 'self.z_spin',
                    '病株 X (m)', '病株 Y (m)', '病株 Z (m)'):
        assert removed not in source
    assert "'工作距离 (m):'" in source
    assert 'self.base_distance_label.setVisible(visible)' in source
    assert 'self.base_distance_spin.setVisible(visible)' in source
