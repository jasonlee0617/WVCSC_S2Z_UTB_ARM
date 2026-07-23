"""Regression checks for the one-path C10 automatic calibration entry."""

from pathlib import Path


PACKAGE_ROOT = Path(__file__).parents[1]
SOURCE_ROOT = PACKAGE_ROOT.parent
LAUNCH_SOURCE = (PACKAGE_ROOT / 'launch' / 'auto_handeye.launch.py').read_text(
    encoding='utf-8')
COLLECTOR_SOURCE = (
    PACKAGE_ROOT / 'wvcsc_calibration' / 'auto_calibration_collector.py'
).read_text(encoding='utf-8')
REAL_CONFIG = (PACKAGE_ROOT / 'config' / 'auto_handeye_alicia.yaml').read_text(
    encoding='utf-8')


def test_auto_handeye_launch_reuses_the_existing_c10_handeye_stack_and_collector():
    assert "'c10_handeye.launch.py'" in LAUNCH_SOURCE
    assert "executable='auto_calibration_collector'" in LAUNCH_SOURCE
    assert "'auto_handeye_alicia.yaml'" in LAUNCH_SOURCE
    assert "'auto_start': False" in LAUNCH_SOURCE
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


def test_real_collector_requires_operator_confirmation_and_exports_both_paths():
    assert 'auto_start: false' in REAL_CONFIG
    assert 'easy_handeye_output_file: ~/.ros2/easy_handeye2/calibrations/wvcsc_c10.calib' \
        in REAL_CONFIG
    assert 'write_calibration_outputs(' in COLLECTOR_SOURCE
    assert 'write_calibration_outputs(' in COLLECTOR_SOURCE.split(
        "self._verify_simulation_ground_truth(handeye)", 1)[1]
    assert 'FollowJointTrajectory' not in COLLECTOR_SOURCE


def test_legacy_alicia_m_calibration_package_is_no_longer_discoverable():
    legacy = SOURCE_ROOT / 'Alicia-M-ROS2' / 'alicia_m_calibration'
    assert not (legacy / 'package.xml').exists()
    assert not (legacy / 'CMakeLists.txt').exists()
    assert not (PACKAGE_ROOT / 'wvcsc_calibration' / 'auto_handeye.py').exists()
