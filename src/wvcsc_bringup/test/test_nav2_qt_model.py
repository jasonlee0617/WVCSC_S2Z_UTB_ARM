import importlib.util
import json
import math
import datetime
from pathlib import Path
from types import SimpleNamespace

import pytest
from builtin_interfaces.msg import Time


SCRIPT = Path(__file__).parents[1] / 'scripts' / 'nav2_qt.py'
LAUNCH = Path(__file__).parents[1] / 'launch' / 'nav2_qt.launch.py'
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

    saved = json.loads(path.read_text(encoding='utf-8'))
    assert saved['schema_version'] == 7
    assert saved['pose_reference'] == (
        nav2_qt.ARM_ANCHOR_POSE_REFERENCE)
    assert 'auto_start' not in saved

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


def test_new_editor_defaults_to_three_second_spray_and_timeline_is_explicit():
    editor = nav2_qt.MissionEditor()
    editor.add_point(_pose(1.0, 0.0), point_type=nav2_qt.POINT_TRANSIT,
                     wide_spray_on_approach=True)
    editor.add_point(
        _pose(2.0, 0.0), tree_y_m=-1.0,
        point_type=nav2_qt.POINT_INSPECT,
        work_side=nav2_qt.WORK_SIDE_RIGHT,
        wide_spray_on_approach=True)

    assert editor.spray_duration == pytest.approx(3.0)
    assert editor.points[1].arm_spray_duration_sec == pytest.approx(3.0)
    assert nav2_qt.route_timeline(editor.points) == (
        '起点 --[广域=ON]--> 1:TRANSIT | '
        '1:TRANSIT --[广域=ON]--> 2:INSPECT 病株=3.0s')


def test_editor_accepts_a_launch_selected_default_spray_duration():
    editor = nav2_qt.MissionEditor(4.0)
    editor.add_point(
        _pose(2.0, 0.0), tree_y_m=-1.0,
        point_type=nav2_qt.POINT_INSPECT,
        work_side=nav2_qt.WORK_SIDE_RIGHT)

    assert editor.spray_duration == pytest.approx(4.0)
    assert editor.points[0].arm_spray_duration_sec == pytest.approx(4.0)
    with pytest.raises(ValueError, match='default_arm_spray_duration_sec'):
        nav2_qt.MissionEditor(10.1)


def test_standard_navigation_points_uses_three_seconds_only_for_inspection():
    path = (SCRIPT.parents[2] / 'wvcsc_mission_manager' / 'config' /
            'navigation_points.json')
    data = json.loads(path.read_text(encoding='utf-8'))

    assert [point['arm_spray_duration_sec'] for point in data['route_points']] == [
        0.0, 3.0, 3.0, 0.0, 0.0]


def test_timestamped_mission_exports_never_reuse_an_existing_filename(tmp_path):
    instant = datetime.datetime(2026, 7, 26, 21, 32, 37)
    first = nav2_qt.timestamped_mission_path(tmp_path, instant)

    assert Path(first).name == 'navigation_points_20260726_213237.json'
    Path(first).touch()
    second = nav2_qt.timestamped_mission_path(tmp_path, instant)

    assert Path(second).name == 'navigation_points_20260726_213237_01.json'


def test_manual_existing_save_path_gets_a_numbered_sibling(tmp_path):
    existing = tmp_path / 'navigation_points.json'
    existing.touch()

    assert nav2_qt.non_overwriting_json_path(existing) == str(
        tmp_path / 'navigation_points_01.json')


def test_qt_launch_exposes_the_ik_recording_admission_policy():
    source = LAUNCH.read_text(encoding='utf-8')

    assert "DeclareLaunchArgument('observation_mode', default_value='joint_presets')" in source
    assert "DeclareLaunchArgument('ik_recording_range_min_m', default_value='0.85')" in source
    assert "DeclareLaunchArgument('ik_recording_range_max_m', default_value='1.45')" in source
    assert "'simulation_parking_clearance_check', default_value='false'" in source
    assert "'default_arm_spray_duration_sec', default_value='3.0'" in source


def test_navigation_qt_keeps_spray_indicators_but_removes_extra_task_controls():
    source = SCRIPT.read_text(encoding='utf-8')

    assert "QTableWidget(0, 3)" in source
    assert "QCheckBox('开启广域喷洒')" in source
    assert "QPushButton('终止任务')" in source
    assert "QCheckBox('显示相机/YOLO画面')" in source
    assert "QLabel('广域喷洒: ● 关闭')" in source
    assert "QLabel('喷嘴喷洒: ● 关闭')" in source
    for obsolete in ('暂停', '继续', '跳过当前', '取消任务', '重置任务'):
        assert f"QPushButton('{obsolete}')" not in source


def test_simulation_parking_preflight_keeps_valid_work_distance_but_rejects_costmap_overlap():
    safe = nav2_qt.WorkPoint(
        _pose(7.02, 0.51), tree_x_m=0.0, tree_y_m=-1.49,
        point_type=nav2_qt.POINT_INSPECT,
        work_side=nav2_qt.WORK_SIDE_RIGHT,
        tree_pose=_pose(7.0, 2.0))
    unsafe = nav2_qt.WorkPoint(
        _pose(7.02, 1.40), tree_x_m=0.0, tree_y_m=-0.60,
        point_type=nav2_qt.POINT_INSPECT,
        work_side=nav2_qt.WORK_SIDE_RIGHT,
        tree_pose=_pose(7.0, 2.0))

    assert nav2_qt.simulation_parking_clearance_m(
        safe.pose, safe.tree_pose) > nav2_qt.SIM_NAV_MIN_PARKING_CLEARANCE_M
    assert nav2_qt.simulation_parking_clearance_error(safe, enabled=True) is None
    assert '仿真停车位过近' in nav2_qt.simulation_parking_clearance_error(
        unsafe, enabled=True)


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
    # v3 stored the vehicle-base goal.  The new editor exposes the fixed
    # Alicia base point instead, while later converting it back for Nav2.
    assert editor.points[0].pose.position.x == pytest.approx(2.6)


@pytest.mark.parametrize('vehicle', [
    _pose(3.0, 0.5, 0.0),
    _pose(3.0, 0.5, math.pi / 2.0),
    _pose(-1.2, 4.3, -0.7),
])
def test_arm_anchor_round_trip_keeps_the_vehicle_heading(vehicle):
    anchor = nav2_qt.arm_anchor_from_vehicle_pose(vehicle)
    reconstructed = nav2_qt.vehicle_pose_from_arm_anchor(anchor)

    assert reconstructed.position.x == pytest.approx(vehicle.position.x)
    assert reconstructed.position.y == pytest.approx(vehicle.position.y)
    assert nav2_qt.pose_yaw(anchor) == pytest.approx(
        nav2_qt.pose_yaw(vehicle))
    assert nav2_qt.pose_yaw(reconstructed) == pytest.approx(
        nav2_qt.pose_yaw(vehicle))


def test_all_operator_point_types_submit_vehicle_base_goals():
    """Every RViz point is an arm-base click, not only inspection points."""
    class _Clock:
        @staticmethod
        def now():
            return SimpleNamespace(to_msg=Time)

    class _RequestBuilder:
        map_frame = 'map'
        observation_mode = 'joint_presets'
        ik_recording_range_min_m = 0.85
        ik_recording_range_max_m = 1.45
        get_clock = staticmethod(_Clock)
        _target_constant = staticmethod(nav2_qt.Nav2QtNode._target_constant)

    start = _pose(1.0, 2.0, 0.3)
    points = [
        nav2_qt.WorkPoint(_pose(2.0, 1.0, 0.1),
                           point_type=nav2_qt.POINT_TRANSIT),
        nav2_qt.WorkPoint(_pose(3.0, 1.5, -0.2), tree_y_m=1.2,
                           point_type=nav2_qt.POINT_INSPECT,
                           work_side=nav2_qt.WORK_SIDE_LEFT),
        nav2_qt.WorkPoint(_pose(4.0, 2.0, 0.4),
                           point_type=nav2_qt.POINT_FINISH),
    ]

    request = nav2_qt.Nav2QtNode.build_manual_request(
        _RequestBuilder(), start, points, False, 'semantic')

    expected_home = nav2_qt.vehicle_pose_from_arm_anchor(start)
    assert request.home_pose.position.x == pytest.approx(expected_home.position.x)
    assert request.home_pose.position.y == pytest.approx(expected_home.position.y)
    assert nav2_qt.pose_yaw(request.home_pose) == pytest.approx(0.3)
    for point, target in zip(points, request.targets):
        expected_goal = nav2_qt.vehicle_pose_from_arm_anchor(point.pose)
        assert target.docking_pose.position.x == pytest.approx(
            expected_goal.position.x)
        assert target.docking_pose.position.y == pytest.approx(
            expected_goal.position.y)
        assert nav2_qt.pose_yaw(target.docking_pose) == pytest.approx(
            nav2_qt.pose_yaw(point.pose))


def test_tree_click_is_converted_to_signed_arm_base_coordinates():
    docking = _pose(0.0, 0.0, 0.0)
    # With the actual Alicia mount yaw=pi, this map location is +Y in
    # alicia_base_link, i.e. the arm's left-side observation mode.
    tree = _pose(-0.4, -1.5, 0.0)
    tree_x, tree_y = nav2_qt.tree_offset_from_docking(docking, tree)
    assert tree_x == pytest.approx(0.0, abs=1e-9)
    assert tree_y == pytest.approx(1.5, abs=1e-9)
    assert nav2_qt.work_side_from_tree_y(tree_y) == nav2_qt.WORK_SIDE_LEFT


def test_tree_markers_show_root_canopy_and_arm_root_distance():
    class _Clock:
        @staticmethod
        def now():
            return SimpleNamespace(to_msg=Time)

    class _MarkerProbe:
        map_frame = 'map'
        get_clock = staticmethod(_Clock)

    root = _pose(7.0, 2.0)
    anchor = _pose(7.0, 0.58)
    root_marker = nav2_qt.Nav2QtNode._tree_root_marker(
        _MarkerProbe(), root, 2)
    canopy_marker = nav2_qt.Nav2QtNode._tree_canopy_marker(
        _MarkerProbe(), root, 2)
    distance_marker = nav2_qt.Nav2QtNode._tree_distance_label(
        _MarkerProbe(), anchor, root, 2)
    label_marker = nav2_qt.Nav2QtNode._tree_label(_MarkerProbe(), root, 2)

    assert root_marker.type == nav2_qt.Marker.CYLINDER
    assert root_marker.scale.x == pytest.approx(0.32)
    assert root_marker.scale.y == pytest.approx(0.32)
    assert canopy_marker.type == nav2_qt.Marker.LINE_STRIP
    assert len(canopy_marker.points) == nav2_qt.TREE_CANOPY_SEGMENTS + 1
    assert canopy_marker.points[0].x == pytest.approx(7.55)
    assert canopy_marker.points[0].y == pytest.approx(2.0)
    assert distance_marker.text == 'ARM-ROOT: 1.42 m'
    assert label_marker.text == 'ROOT\nCANOPY r=0.55m'


def test_opencv_qt_plugin_override_is_removed_without_touching_other_paths():
    environment = {
        'QT_QPA_PLATFORM_PLUGIN_PATH': (
            '/home/eisa/.local/lib/python3.10/site-packages/cv2/qt/plugins'),
        'QT_QPA_FONTDIR': (
            '/home/eisa/.local/lib/python3.10/site-packages/cv2/qt/fonts'),
        'QT_PLUGIN_PATH': '/opt/custom/plugins',
    }
    nav2_qt.remove_opencv_qt_plugin_override(environment)
    assert 'QT_QPA_PLATFORM_PLUGIN_PATH' not in environment
    assert 'QT_QPA_FONTDIR' not in environment
    assert environment['QT_PLUGIN_PATH'] == '/opt/custom/plugins'


def test_inspect_side_mismatch_is_rejected_before_mission_submission():
    point = nav2_qt.WorkPoint(
        _pose(0.0, 0.0), tree_y_m=-1.0,
        point_type=nav2_qt.POINT_INSPECT,
        work_side=nav2_qt.WORK_SIDE_LEFT)
    assert '不一致' in nav2_qt.valid_work_side(point)


def test_ik_recording_range_has_a_margin_but_joint_presets_bypass_it():
    point = nav2_qt.WorkPoint(
        _pose(0.0, 0.0), tree_x_m=0.0, tree_y_m=-1.50,
        point_type=nav2_qt.POINT_INSPECT,
        work_side=nav2_qt.WORK_SIDE_RIGHT)

    error = nav2_qt.ik_recording_range_error(point, 'ik', 0.85, 1.45)
    assert error is not None
    assert '1.50' in error
    assert nav2_qt.ik_recording_range_error(
        point, 'joint_presets', 0.85, 1.45) is None


def test_ik_request_rejects_an_out_of_range_inspection_point():
    class _Clock:
        @staticmethod
        def now():
            return SimpleNamespace(to_msg=Time)

    class _RequestBuilder:
        map_frame = 'map'
        observation_mode = 'ik'
        ik_recording_range_min_m = 0.85
        ik_recording_range_max_m = 1.45
        get_clock = staticmethod(_Clock)
        _target_constant = staticmethod(nav2_qt.Nav2QtNode._target_constant)

    point = nav2_qt.WorkPoint(
        _pose(3.0, 0.5), tree_x_m=0.0, tree_y_m=-1.50,
        point_type=nav2_qt.POINT_INSPECT,
        work_side=nav2_qt.WORK_SIDE_RIGHT)
    with pytest.raises(ValueError, match='超出录入范围'):
        nav2_qt.Nav2QtNode.build_manual_request(
            _RequestBuilder(), _pose(0.0, 0.0), [point],
            False, 'ik_range')


def test_vehicle_route_marker_connects_converted_vehicle_goals():
    class _Clock:
        @staticmethod
        def now():
            return SimpleNamespace(to_msg=Time)

    class _MarkerProbe:
        map_frame = 'map'
        get_clock = staticmethod(_Clock)

    anchors = [_pose(0.0, 0.0), _pose(3.0, 0.5), _pose(6.0, 0.5)]
    marker = nav2_qt.Nav2QtNode._vehicle_route_marker(
        _MarkerProbe(), anchors)

    assert marker.type == nav2_qt.Marker.LINE_STRIP
    assert marker.ns == 'manual_vehicle_route'
    assert len(marker.points) == 3
    expected = [nav2_qt.vehicle_pose_from_arm_anchor(anchor)
                for anchor in anchors]
    for actual, vehicle_pose in zip(marker.points, expected):
        assert actual.x == pytest.approx(vehicle_pose.position.x)
        assert actual.y == pytest.approx(vehicle_pose.position.y)


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

    def currentData(self):
        return nav2_qt.POINT_TRANSIT


class _GuiProbe:
    ACTIVE = nav2_qt.Nav2Gui.ACTIVE
    TERMINAL = nav2_qt.Nav2Gui.TERMINAL
    _refresh = nav2_qt.Nav2Gui._refresh
    _start_task = nav2_qt.Nav2Gui._start_task
    _copy_work_point = staticmethod(nav2_qt.Nav2Gui._copy_work_point)
    _record_start = nav2_qt.Nav2Gui._record_start
    _relocalize_and_clear = nav2_qt.Nav2Gui._relocalize_and_clear
    _update_record_point_button = nav2_qt.Nav2Gui._update_record_point_button


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
    gui.pending = False
    gui.required_initial_pose_sequence = 0
    gui.relocalization_ready = True
    for name in (
            'record_start_button', 'record_point_button', 'start_task_button',
            'delete_button', 'up_button', 'down_button',
            'clear_button', 'save_button', 'load_button', 'home_button',
            'abort_home_button', 'point_type_combo', 'wide_spray_checkbox',
            'candidate_label', 'status_label',
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


def test_gui_task_button_accepts_one_or_more_queued_targets():
    editor = nav2_qt.MissionEditor()
    editor.start_pose = _pose(0.0, 0.0)
    gui = _gui(editor)

    gui._refresh()
    assert not gui.start_task_button.enabled

    editor.add_point(_pose(3.0, 0.5))
    gui._refresh()
    assert gui.start_task_button.enabled

    editor.add_point(_pose(5.0, -0.5))
    gui._refresh()
    assert gui.start_task_button.enabled


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
    expected_anchor = nav2_qt.arm_anchor_from_vehicle_pose(
        _pose(7.0, 8.0, 0.3))
    assert editor.start_pose.position.x == pytest.approx(
        expected_anchor.position.x)
    assert editor.start_pose.position.y == pytest.approx(
        expected_anchor.position.y)
    assert nav2_qt.pose_yaw(editor.start_pose) == pytest.approx(0.3)


def test_task_uses_all_queued_targets_and_keeps_the_editor_list():
    editor = nav2_qt.MissionEditor()
    editor.start_pose = _pose(0.0, 0.0)
    editor.add_point(_pose(3.0, 0.5, 0.3), 0.0, -1.5)
    gui = _gui(editor)
    submitted = {}
    gui._submit_manual = lambda points, prefix: submitted.update(
        points=points, prefix=prefix)

    gui._start_task()
    assert submitted['prefix'] == 'task'
    assert submitted['points'][0].tree_y_m == -1.5
    assert math.isclose(submitted['points'][0].pose.position.x, 3.0)
    assert len(editor.points) == 1
