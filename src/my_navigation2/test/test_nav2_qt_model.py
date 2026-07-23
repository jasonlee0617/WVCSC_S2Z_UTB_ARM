import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace

import pytest


SCRIPT = Path(__file__).parents[1] / 'scripts' / 'nav2_qt.py'
SPEC = importlib.util.spec_from_file_location('nav2_qt', SCRIPT)
nav2_qt = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(nav2_qt)


def _pose(x, y, yaw=0.0):
    pose = nav2_qt.Pose()
    pose.position.x = x
    pose.position.y = y
    pose.orientation.z = nav2_qt.math.sin(yaw / 2.0)
    pose.orientation.w = nav2_qt.math.cos(yaw / 2.0)
    return pose


def test_editor_round_trip_preserves_signed_tree_xy(tmp_path):
    editor = nav2_qt.MissionEditor()
    editor.start_pose = _pose(0.0, 0.0)
    editor.spray_duration = 2.5
    editor.return_home_after_finish = True
    editor.add_point(_pose(3.0, 0.5, 0.3), 0.0, 1.5)
    editor.add_point(_pose(5.0, -0.5, -0.2), 0.1, -1.5)
    path = tmp_path / 'manual_mission.json'
    editor.save(path)

    loaded = nav2_qt.MissionEditor()
    loaded.load(path)
    assert loaded.spray_duration == 2.5
    assert loaded.return_home_after_finish
    assert [(point.tree_x_m, point.tree_y_m) for point in loaded.points] == [
        (0.0, 1.5), (0.1, -1.5)]
    assert math.isclose(nav2_qt.pose_yaw(loaded.points[0].pose), 0.3)


def test_editor_rejects_legacy_file_without_measured_tree_xy(tmp_path):
    path = tmp_path / 'legacy.json'
    path.write_text(
        '{"point1": {"position": {"x": 0, "y": 0, "z": 0}, '
        '"orientation": {"x": 0, "y": 0, "z": 0, "w": 1}}, '
        '"point2": {"position": {"x": 4, "y": -1, "z": 0}, '
        '"orientation": {"x": 0, "y": 0, "z": 0, "w": 1}}}',
        encoding='utf-8')
    editor = nav2_qt.MissionEditor()
    with pytest.raises(ValueError, match='legacy navigation file'):
        editor.load(path)


class _Widget:
    def __init__(self):
        self.enabled = None
        self.text = ''

    def setEnabled(self, value):
        assert type(value) is bool
        self.enabled = value

    def setText(self, value):
        self.text = value


class _GuiProbe:
    ACTIVE = nav2_qt.Nav2Gui.ACTIVE
    TERMINAL = nav2_qt.Nav2Gui.TERMINAL
    _refresh = nav2_qt.Nav2Gui._refresh
    _start_single = nav2_qt.Nav2Gui._start_single
    _consume_completed_single_target = (
        nav2_qt.Nav2Gui._consume_completed_single_target)


def _gui(editor):
    gui = _GuiProbe()
    gui.node = SimpleNamespace(
        goal_sequence=0,
        latest_goal_pose=None,
        status=None,
    )
    gui.editor = editor
    gui.candidate = None
    gui.candidate_sequence = 0
    gui.consumed_goal_sequence = 0
    gui.single_mission_id = None
    gui.pending = False
    for name in (
            'record_start_button', 'add_point_button', 'single_button',
            'multi_button', 'delete_button', 'up_button', 'down_button',
            'clear_button', 'save_button', 'load_button', 'pause_button',
            'resume_button', 'skip_button', 'cancel_button', 'home_button',
            'reset_button', 'candidate_label', 'status_label'):
        setattr(gui, name, _Widget())
    gui._publish_markers = lambda: None
    gui._update_table = lambda: None
    gui._log = lambda _message: None
    return gui


def test_gui_buttons_require_one_or_two_queued_targets():
    editor = nav2_qt.MissionEditor()
    editor.start_pose = _pose(0.0, 0.0)
    gui = _gui(editor)

    gui._refresh()
    assert not gui.single_button.enabled
    assert not gui.multi_button.enabled

    editor.add_point(_pose(3.0, 0.5))
    gui._refresh()
    assert gui.single_button.enabled
    assert not gui.multi_button.enabled

    editor.add_point(_pose(5.0, -0.5))
    gui._refresh()
    assert not gui.single_button.enabled
    assert gui.multi_button.enabled


def test_single_uses_queued_target_and_removes_it_only_after_completion():
    editor = nav2_qt.MissionEditor()
    editor.start_pose = _pose(0.0, 0.0)
    editor.add_point(_pose(3.0, 0.5, 0.3), 0.0, -1.5)
    gui = _gui(editor)
    submitted = {}
    gui._submit_manual = lambda points, prefix: submitted.update(
        points=points, prefix=prefix)

    gui._start_single()
    assert submitted['prefix'] == 'single'
    assert submitted['points'][0].tree_y_m == -1.5
    assert math.isclose(submitted['points'][0].pose.position.x, 3.0)

    gui.single_mission_id = 'manual_single_01'
    gui.node.status = SimpleNamespace(
        mission_id='manual_single_01', completed_targets=0)
    gui._consume_completed_single_target()
    assert len(editor.points) == 1

    gui.node.status.completed_targets = 1
    gui._consume_completed_single_target()
    assert editor.points == []
    assert gui.single_mission_id is None
