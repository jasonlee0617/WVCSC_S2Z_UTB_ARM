from wvcsc_rgb_vision.core import AlignmentTracker


def test_alignment_requires_consecutive_centered_fresh_frames():
    tracker = AlignmentTracker(stable_frames=3, stale_timeout=0.5)
    for stamp in (1.0, 1.1):
        assert tracker.update(
            stamp, 'mission', 'tree_01', True, 0.9,
            640.0, 360.0, 1280, 720)
    assert tracker.status(1.1, 'mission', 'tree_01')[0] == tracker.WAITING
    tracker.update(
        1.2, 'mission', 'tree_01', True, 0.9,
        650.0, 370.0, 1280, 720)
    status, error_u, error_v, frames = tracker.status(
        1.2, 'mission', 'tree_01')
    assert status == tracker.ALIGNED
    assert (error_u, error_v, frames) == (10.0, 10.0, 3)


def test_off_center_wrong_tree_and_stale_samples_do_not_align():
    tracker = AlignmentTracker(stable_frames=1, stale_timeout=0.5)
    assert not tracker.update(
        1.0, 'mission', '', True, 0.9, 640.0, 360.0, 1280, 720)
    tracker.update(
        1.0, 'mission', 'tree_02', True, 0.9,
        700.0, 360.0, 1280, 720)
    assert tracker.status(1.0, 'mission', 'tree_01')[0] == tracker.WAITING
    assert tracker.status(1.6, 'mission', 'tree_02')[0] == tracker.STALE


def test_reset_requires_new_stable_frames():
    tracker = AlignmentTracker(stable_frames=2)
    for stamp in (1.0, 1.1):
        assert tracker.update(
            stamp, 'mission', 'tree', True, 0.9,
            320.0, 240.0, 640, 480)
    assert tracker.status(1.1, 'mission', 'tree')[0] == tracker.ALIGNED

    tracker.reset()

    assert tracker.status(1.2, 'mission', 'tree')[0] == tracker.STALE
    assert tracker.update(
        1.3, 'mission', 'tree', True, 0.9,
        320.0, 240.0, 640, 480)
    assert tracker.status(1.3, 'mission', 'tree')[0] == tracker.WAITING


def test_stable_count_is_scoped_to_mission_and_tree():
    tracker = AlignmentTracker(stable_frames=2)
    tracker.update(
        1.0, 'old_mission', 'tree', True, 0.9,
        320.0, 240.0, 640, 480)
    tracker.update(
        1.1, 'new_mission', 'tree', True, 0.9,
        320.0, 240.0, 640, 480)

    status, _error_u, _error_v, frames = tracker.status(
        1.1, 'new_mission', 'tree')
    assert status == tracker.WAITING
    assert frames == 1
