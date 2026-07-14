from types import SimpleNamespace

from wvcsc_arm_task.trajectory_validation import valid_retimed_trajectory


def _trajectory(times, position_count=2):
    return SimpleNamespace(
        joint_names=['j1', 'j2'],
        points=[
            SimpleNamespace(
                positions=[0.0] * position_count,
                time_from_start=SimpleNamespace(sec=t // 1_000_000_000,
                                                nanosec=t % 1_000_000_000),
            )
            for t in times
        ],
    )


def test_accepts_strictly_increasing_timestamps():
    assert valid_retimed_trajectory(_trajectory([0, 100_000_000, 200_000_000]))


def test_rejects_empty_repeated_or_malformed_trajectory():
    assert not valid_retimed_trajectory(_trajectory([]))
    assert not valid_retimed_trajectory(_trajectory([10, 10]))
    assert not valid_retimed_trajectory(_trajectory([10], position_count=1))

