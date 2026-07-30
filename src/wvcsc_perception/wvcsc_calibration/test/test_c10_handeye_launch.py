from pathlib import Path


LAUNCH_SOURCE = (
    Path(__file__).parents[1] / 'launch' / 'c10_handeye.launch.py'
).read_text(encoding='utf-8')
EVALUATE_SOURCE = (
    Path(__file__).parents[1] / 'launch' / 'evaluate.launch.py'
).read_text(encoding='utf-8')


def test_real_handeye_launch_starts_aruco_debug_overlay():
    assert "executable='visualize_aruco_marker'" in LAUNCH_SOURCE
    assert "'/camera/color/image_raw'" in LAUNCH_SOURCE
    assert "'/camera/color/camera_info'" in LAUNCH_SOURCE
    assert "'/calibration/aruco_debug_image'" in LAUNCH_SOURCE
    assert "'use_sim_time': False" in LAUNCH_SOURCE


def test_real_handeye_launch_logs_key_nodes_to_files():
    assert "executable='marker_tf'" in LAUNCH_SOURCE
    assert "executable='handeye_server'" in LAUNCH_SOURCE
    assert LAUNCH_SOURCE.count("output='both'") >= 4


def test_evaluate_launch_resolves_one_shared_calibration_path():
    assert ('from wvcsc_bringup.handeye_calibration_paths import '
            'resolve_handeye_calibration') in EVALUATE_SOURCE
    assert 'def _latest_handeye_path' not in EVALUATE_SOURCE
    assert EVALUATE_SOURCE.count('resolve_handeye_calibration(') == 1
    assert EVALUATE_SOURCE.count('OpaqueFunction(function=_evaluate_actions)') == 1
    assert EVALUATE_SOURCE.count("'calibration_file': value") == 2
