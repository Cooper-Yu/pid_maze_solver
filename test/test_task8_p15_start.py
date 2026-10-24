"""Fault and cancellation tests for the no-motion P15 initialization loop."""

from dataclasses import dataclass
from pathlib import Path
import sys
from types import SimpleNamespace as N

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
import task8_p15_start as start  # noqa: E402


@dataclass
class Pose:
    x: float
    y: float
    yaw: float


def fixture_runner(monkeypatch, ready=True, publishers=0):
    clock = [100.0]
    events = []
    monkeypatch.setattr(start.time, 'monotonic', lambda: clock[0])
    monkeypatch.setattr(start, 'validate_source', lambda *_args: {})
    monkeypatch.setattr(start, 'load_reference', lambda *_args: None)
    monkeypatch.setitem(sys.modules, 'action_plan', N(Pose=Pose))
    node = N(
        count_publishers=lambda _topic: publishers,
        get_clock=lambda: N(now=lambda: N(nanoseconds=int(clock[0] * 1e9))),
    )
    runner = N(
        node=node,
        frame='odom',
        twist_frame='base_link',
        task8_emit=lambda event, **data: events.append((event, data)),
    )

    def pump():
        clock[0] += 0.1
        runner.stamp = int(clock[0] * 1e9)
        runner.latest = (Pose(10, 20, 1.5), clock[0], 0, 0)

    pause = N(
        original_pump=pump,
        check_faults=lambda: None,
        ready=lambda: ready,
        sources={s: N(sample={'stopped': True}) for s in ('raw', 'filtered')},
    )
    runner.feedback_pause = pause

    def sample(_runner, _reference):
        return dict(
            time=clock[0],
            scan_stamp=runner.stamp,
            odom_stamp=runner.stamp,
            odom=dict(x=10, y=20, yaw=1.5),
            estimate=dict(x=-0.014, y=-0.008, yaw=-0.006),
        )

    monkeypatch.setattr(start, 'current_sample', sample)
    return runner, N(check=lambda: None), events


def test_stopped_current_epoch_initialization_preserves_nonzero_terminal_offset(monkeypatch):
    runner, cancel, events = fixture_runner(monkeypatch)
    origin = start.initialize(runner, 'book', 'bag', (0, 0), cancel)
    from task8_p15_localization import compose

    recovered = compose(vars(origin), dict(x=-0.014, y=-0.008, yaw=-0.006))
    assert recovered == pytest.approx(dict(x=10, y=20, yaw=1.5))
    assert events[-1][0] == 'p15_start_verified'
    assert events[-1][1]['motion_published'] is False


def test_missing_feedback_waits_then_times_out_without_publisher(monkeypatch):
    runner, cancel, events = fixture_runner(monkeypatch, ready=False)
    with pytest.raises(RuntimeError, match='wait expired'):
        start.initialize(runner, 'book', 'bag', (0, 0), cancel)
    assert events == []


def test_cancellation_is_not_swallowed_by_wait(monkeypatch):
    runner, cancel, events = fixture_runner(monkeypatch)

    def cancel_now():
        raise KeyboardInterrupt('cancel')

    cancel.check = cancel_now
    with pytest.raises(KeyboardInterrupt):
        start.initialize(runner, 'book', 'bag', (0, 0), cancel)
    assert events == []


def test_competing_publisher_rejected(monkeypatch):
    runner, cancel, events = fixture_runner(monkeypatch, publishers=1)
    with pytest.raises(RuntimeError, match='no velocity publishers'):
        start.initialize(runner, 'book', 'bag', (0, 0), cancel)
    assert events == []


def test_changed_odom_identity_is_fatal(monkeypatch):
    runner, cancel, events = fixture_runner(monkeypatch)

    def fault():
        raise RuntimeError('GID changed')

    runner.feedback_pause.check_faults = fault
    with pytest.raises(RuntimeError, match='GID changed'):
        start.initialize(runner, 'book', 'bag', (0, 0), cancel)
    assert events == []


def test_missing_walls_never_authorize_start(monkeypatch):
    runner, cancel, events = fixture_runner(monkeypatch)

    def missing(*_args):
        raise ValueError('No wall')

    monkeypatch.setattr(start, 'current_sample', missing)
    with pytest.raises(RuntimeError, match='No wall'):
        start.initialize(runner, 'book', 'bag', (0, 0), cancel)
    assert events == []


@pytest.mark.parametrize(
    'changed',
    [
        ([0.021, 0, 0.17345], [0, 0, 1, 0]),
        ([0.02, 0, 0.17345], [0, 0, 0, 1]),
        ([0.02, 0, 0.17345], [0, 0, 0.5, 0]),
        ([float('nan'), 0, 0.17345], [0, 0, 1, 0]),
    ],
)
def test_changed_or_invalid_mount_rejected(changed):
    with pytest.raises(ValueError):
        start.require_same_mount(([0.02, 0, 0.17345], [0, 0, 1, 0]), changed)


def test_equivalent_quaternion_sign_accepted():
    start.require_same_mount(
        ([0.02, 0, 0.17345], [0, 0, 1, 0]), ([0.02, 0, 0.17345], [0, 0, -1, 0])
    )
