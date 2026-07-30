"""Regression checks for the one-path C10 automatic calibration entry."""

from pathlib import Path


PACKAGE_ROOT = Path(__file__).parents[1]
LAUNCH_SOURCE = (PACKAGE_ROOT / 'launch' / 'auto_handeye.launch.py').read_text(
    encoding='utf-8')
COLLECTOR_SOURCE = (
    PACKAGE_ROOT / 'wvcsc_calibration' / 'auto_calibration_collector.py'
).read_text(encoding='utf-8')
REAL_CONFIG = (PACKAGE_ROOT / 'config' / 'real' /
               'auto_handeye_alicia.yaml').read_text(encoding='utf-8')
QT_SOURCE = (PACKAGE_ROOT / 'wvcsc_calibration' / 'calibration_qt.py').read_text(
    encoding='utf-8')


def test_auto_handeye_launch_starts_one_embedded_qt_collector_by_default():
    assert "'c10_handeye.launch.py'" in LAUNCH_SOURCE
    assert "executable='auto_calibration_collector'" not in LAUNCH_SOURCE
    assert "executable='calibration_qt'" in LAUNCH_SOURCE
    assert "'use_calibration_qt', default_value='true'" in LAUNCH_SOURCE
    assert "'config', 'real', 'auto_handeye_alicia.yaml'" in LAUNCH_SOURCE
    assert "SetEnvironmentVariable('PYTHONNOUSERSITE', '1')" in LAUNCH_SOURCE
    assert "executable='auto_handeye'" not in LAUNCH_SOURCE
    assert 'pymoveit2' not in LAUNCH_SOURCE


def test_automatic_entry_uses_only_the_c10_calibration_frame_contract():
    for required in (
            'alicia_base_link', 'tool0', 'camera_color_optical_frame',
            'calibration_aruco', '/camera/color/image_raw',
            '/camera/color/camera_info'):
        assert required in REAL_CONFIG or required in (
            PACKAGE_ROOT / 'launch' / 'c10_handeye.launch.py'
        ).read_text(encoding='utf-8')
    assert '/camera/camera/' not in LAUNCH_SOURCE
    assert 'd405' not in LAUNCH_SOURCE.lower()


def test_real_collector_uses_real_profile_and_archives_native_results():
    assert 'auto_start: false' in REAL_CONFIG
    assert 'calibration_output_dir:' in REAL_CONFIG
    assert 'calibration_file_prefix: c10_handeye' in REAL_CONFIG
    assert 'calibration_simulation: false' in REAL_CONFIG
    assert '/config/real' in REAL_CONFIG
    assert 'easy_handeye2/calibrations' not in REAL_CONFIG
    assert 'write_calibration_outputs(' in COLLECTOR_SOURCE
    assert 'write_calibration_outputs(' in COLLECTOR_SOURCE.split(
        "self._verify_simulation_ground_truth(handeye)", 1)[1]
    assert 'FollowJointTrajectory' not in COLLECTOR_SOURCE
    assert 'write_sample_archive(' in COLLECTOR_SOURCE
    assert "'/calibration/prepare'" in COLLECTOR_SOURCE
    assert "'/calibration/collect'" in COLLECTOR_SOURCE
    assert "'/calibration/state'" in COLLECTOR_SOURCE


def test_collector_copies_the_legacy_sequence_without_a_runtime_dependency():
    assert 'fixed_joint_samples' in COLLECTOR_SOURCE
    assert 'alicia_m_calibration' not in COLLECTOR_SOURCE
    assert not (PACKAGE_ROOT / 'wvcsc_calibration' / 'auto_handeye.py').exists()


def test_qt_reuses_shared_image_panel_and_state_driven_home_recovery():
    assert 'RosImagePanel' in QT_SOURCE
    assert "'/calibration/aruco_debug_image'" in QT_SOURCE
    assert "'/camera/color/image_raw'" in QT_SOURCE
    assert "state == 'STOPPED_LOCKED'" in QT_SOURCE
    assert "String(data='reset')" in QT_SOURCE
    assert 'QTimer.singleShot' not in QT_SOURCE


def test_qt_does_not_replace_rclpy_service_or_client_entity_collections():
    assert 'self._services =' not in QT_SOURCE
    assert 'self._clients =' not in QT_SOURCE
    assert 'self._calibration_clients =' in QT_SOURCE


def test_qt_restores_latched_states_received_before_gui_signal_connection():
    assert 'self.latest_calibration_state = state' in QT_SOURCE
    assert 'self.latest_motion_state = state' in QT_SOURCE
    gui_init = QT_SOURCE.split('class CalibrationQt(QWidget):', 1)[1].split(
        '    def _build_ui(self):', 1)[0]
    assert gui_init.index('signals.calibration_state.connect') < gui_init.index(
        'self._set_calibration_state(self._node.latest_calibration_state)')
    assert gui_init.index('signals.motion_state.connect') < gui_init.index(
        'self._set_motion_state(self._node.latest_motion_state)')


def test_qt_keeps_simulation_truth_only_in_the_terminal_log():
    assert 'truth_label' not in QT_SOURCE
    assert "'[CALIBRATION][GROUND_TRUTH]'" not in QT_SOURCE
    assert 'self.log_area.append' in QT_SOURCE
    assert 'str(message.msg)' in QT_SOURCE


def test_real_profile_does_not_enable_simulation_truth_check():
    assert 'ground_truth_check_enabled' not in REAL_CONFIG
    assert 'calibration_simulation: false' in REAL_CONFIG
