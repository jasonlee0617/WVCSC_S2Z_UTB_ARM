import importlib.util
from pathlib import Path


SCRIPT = Path(__file__).parents[1] / 'scripts' / 'arm_spray_test_qt.py'
SPEC = importlib.util.spec_from_file_location('arm_spray_test_qt', SCRIPT)
arm_spray_test_qt = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(arm_spray_test_qt)


def test_single_target_goal_uses_the_selected_arm_frame_and_values():
    goal = arm_spray_test_qt.build_spray_goal(
        'mission_1', 'corn_01', 'alicia_base_link',
        0.1, 1.5, 0.2, 3.0)
    assert goal.mission_id == 'mission_1'
    assert goal.tree_id == 'corn_01'
    assert goal.tree_hint.header.frame_id == 'alicia_base_link'
    assert (goal.tree_hint.point.x, goal.tree_hint.point.y,
            goal.tree_hint.point.z, goal.spray_duration) == (0.1, 1.5, 0.2, 3.0)
