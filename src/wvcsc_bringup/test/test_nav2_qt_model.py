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
    editor.add_point(
        _pose(3.0, 0.5, 0.3), 0.0, 1.5,
        point_type=nav2_qt.POINT_INSPECT,
        work_side=nav2_qt.WORK_SIDE_LEFT,
        wide_spray_on_approach=True,
        arm_spray_duration_sec=3.0,
        dwell_time_sec=1.5,
        tree_pose=_pose(2.6, -1.0))
    editor.add_point(
        _pose(5.0, -0.5, -0.2),
        point_type=nav2_qt.POINT_FINISH,
        wide_spray_on_approach=False)
    path = tmp_path / 'manual_mission.json'
    editor.save(path)

    loaded = nav2_qt.MissionEditor()
    loaded.load(path)
    assert loaded.spray_duration == 2.5
    assert loaded.return_home_after_finish
    assert [(point.tree_x_m, point.tree_y_m) for point in loaded.points] == [
        (0.0, 1.5), (0.0, 0.0)]
    assert math.isclose(nav2_qt.pose_yaw(loaded.points[0].pose), 0.3)
    assert loaded.points[0].point_type == nav2_qt.POINT_INSPECT
    assert loaded.points[0].work_side == nav2_qt.WORK_SIDE_LEFT
    assert loaded.points[0].wide_spray_on_approach
    assert loaded.points[0].arm_spray_duration_sec == 3.0
    assert loaded.points[0].dwell_time_sec == 1.5
    assert loaded.points[0].tree_pose is not None
    assert loaded.points[1].point_type == nav2_qt.POINT_FINISH


def test_editor_migrates_schema_v3_with_a_visible_warning(tmp_path):
    path = tmp_path / 'legacy.json'
    path.write_text(
        '{"schema_version": 3, "spray_duration": 2.0, "targets": ['
        '{"pose": {"position": {"x": 3, "y": 0, "z": 0}, '
        '"orientation": {"x": 0, "y": 0, "z": 0, "w": 1}}, '
        '"tree_x_m": 0.1, "tree_y_m": -1.5}]}',
        encoding='utf-8')
    editor = nav2_qt.MissionEditor()
    editor.load(path)
    assert editor.load_warning
    assert editor.points[0].point_type == nav2_qt.POINT_INSPECT
    assert editor.points[0].work_side == nav2_qt.WORK_SIDE_RIGHT
    assert editor.points[0].tree_pose is None


def test_tree_click_is_converted_to_signed_arm_base_coordinates():
    docking = _pose(0.0, 0.0, 0.0)
    # With the actual Alicia mount yaw=pi, this map location is +Y in
    # alicia_base_link, i.e. the arm's left-side observation mode.
    tree = _pose(-0.4, -1.5, 0.0)
    tree_x, tree_y = nav2_qt.tree_offset_from_docking(docking, tree)
    assert tree_x == pytest.approx(0.0, abs=1e-9)
    assert tree_y == pytest.approx(1.5, abs=1e-9)
    assert nav2_qt.work_side_from_tree_y(tree_y) == nav2_qt.WORK_SIDE_LEFT


def test_inspect_side_mismatch_is_rejected_before_mission_submission():
    point = nav2_qt.WorkPoint(
        _pose(0.0, 0.0), tree_y_m=-1.0,
        point_type=nav2_qt.POINT_INSPECT,
        work_side=nav2_qt.WORK_SIDE_LEFT)
    assert '不一致' in nav2_qt.valid_work_side(point)


def test_finish_point_forces_wide_spray_off(tmp_path):
    editor = nav2_qt.MissionEditor()
    editor.add_point(
        _pose(2.0, 0.0), point_type=nav2_qt.POINT_FINISH,
        wide_spray_on_approach=True)
    assert not editor.points[0].wide_spray_on_approach

    path = tmp_path / 'finish.json'
    editor.save(path)
    loaded = nav2_qt.MissionEditor()
    loaded.load(path)
    assert not loaded.points[0].wide_spray_on_approach


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
    _copy_work_point = staticmethod(nav2_qt.Nav2Gui._copy_work_point)
    _consume_completed_single_target = (
        nav2_qt.Nav2Gui._consume_completed_single_target)
    _record_start = nav2_qt.Nav2Gui._record_start
    _relocalize_and_clear = nav2_qt.Nav2Gui._relocalize_and_clear


def _gui(editor):
    gui = _GuiProbe()
    gui.node = SimpleNamespace(
        goal_sequence=0,
        latest_goal_pose=None,
        initial_pose_sequence=0,
        latest_initial_pose=None,
        status=None,
    )
    gui.editor = editor
    gui.candidate = None
    gui.candidate_sequence = 0
    gui.consumed_goal_sequence = 0
    gui.pending_dock_pose = None
    gui.pending_dock_sequence = 0
    gui.single_mission_id = None
    gui.pending = False
    gui.required_initial_pose_sequence = 0
    gui.relocalization_ready = True
    for name in (
            'record_start_button', 'add_point_button', 'single_button',
            'multi_button', 'delete_button', 'up_button', 'down_button',
            'clear_button', 'save_button', 'load_button', 'pause_button',
            'resume_button', 'skip_button', 'cancel_button', 'home_button',
            'reset_button', 'abort_home_button', 'point_type_combo',
            'capture_tree_button', 'candidate_label', 'status_label',
            'start_label', 'capture_label', 'relocalize_button'):
        setattr(gui, name, _Widget())
    gui._publish_markers = lambda: None
    gui._update_table = lambda: None
    gui._log = lambda _message: None
    return gui


class _ImmediateFuture:
    @staticmethod
    def result():
        return None

    def add_done_callback(self, callback):
        callback(self)


class _ServiceClient:
    @staticmethod
    def service_is_ready():
        return True

    @staticmethod
    def call_async(_request):
        return _ImmediateFuture()


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


def test_relocalize_clears_editor_and_requires_a_new_tf_backed_start():
    editor = nav2_qt.MissionEditor()
    editor.start_pose = _pose(1.0, 2.0)
    editor.add_point(_pose(3.0, 0.5))
    gui = _gui(editor)
    gui.node.service_clients = {
        'reinitialize_global_localization': _ServiceClient(),
    }
    gui.node.current_pose = lambda: _pose(7.0, 8.0, 0.3)

    gui._relocalize_and_clear()

    assert editor.start_pose is None
    assert editor.points == []
    assert gui.required_initial_pose_sequence == 0
    assert gui.relocalization_ready
    gui._refresh()
    assert not gui.record_start_button.enabled

    gui.node.initial_pose_sequence = 1
    gui.node.latest_initial_pose = _pose(0.0, 0.0)
    gui._refresh()
    assert gui.record_start_button.enabled
    gui._record_start()
    assert editor.start_pose.position.x == pytest.approx(7.0)
    assert editor.start_pose.position.y == pytest.approx(8.0)


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
