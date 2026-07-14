from types import SimpleNamespace

from wvcsc_web_ui.state import SnapshotStore


def _header(frame='map'):
    return SimpleNamespace(
        frame_id=frame,
        stamp=SimpleNamespace(sec=12, nanosec=34),
    )


def test_store_builds_json_safe_mission_snapshot():
    store = SnapshotStore()
    tree = SimpleNamespace(
        tree_id='tree_01', confidence=0.96,
        position=SimpleNamespace(x=3.0, y=2.0, z=0.0),
        spray_side='left', spray_duration=2.0, evidence_uri='evidence.jpg')
    mission = SimpleNamespace(
        header=_header(), mission_id='demo', source_mode='mock', trees=[tree])
    store.update_mission(mission)

    snapshot = store.snapshot()

    assert snapshot['generation'] == 1
    assert not snapshot['connected']
    assert snapshot['mission']['trees'][0]['position']['x'] == 3.0
    assert snapshot['mission']['source_mode'] == 'mock'


def test_status_marks_ros_connected_and_snapshot_is_a_copy():
    store = SnapshotStore()
    status = SimpleNamespace(
        header=_header(), mission_id='demo', state=3, state_text='NAVIGATING',
        current_tree_id='tree_01', current_index=0, total_targets=2,
        completed_targets=0, last_error='', nav_goal_active=True,
        arm_goal_active=False, skipped_targets=0,
        target_statuses=[SimpleNamespace(
            tree_id='tree_01', state=1, state_text='CURRENT', message='')])
    store.update_status(status)

    first = store.snapshot()
    first['status']['state_text'] = 'CORRUPTED'
    second = store.snapshot()

    assert second['connected']
    assert second['status']['state_text'] == 'NAVIGATING'
    assert second['status']['target_statuses'][0]['state_text'] == 'CURRENT'


def test_store_exposes_mission_plan_for_map_rendering():
    store = SnapshotStore()
    target = SimpleNamespace(
        tree_id='tree_01', position=SimpleNamespace(x=3.0, y=2.0, z=0.0))
    pose = SimpleNamespace(
        position=SimpleNamespace(x=3.0, y=0.5, z=0.0),
        orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0))
    item = SimpleNamespace(target=target, docking_pose=pose)
    plan = SimpleNamespace(
        header=_header(), mission_id='demo', targets=[item],
        return_home_after_finish=True, home_pose=pose)

    store.update_plan(plan)

    snapshot = store.snapshot()
    assert snapshot['plan']['targets'][0]['docking_pose']['position']['y'] == 0.5
    assert snapshot['plan']['return_home_after_finish']
