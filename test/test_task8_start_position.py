"""Position acquisition keeps the original final gate and bounds each PID step."""

import io
import math
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from task8_runner import backend_imports

backend_imports()
from action_plan import Pose  # noqa: E402
import task8_start_position as position  # noqa: E402
from task8_p15_localization import qualify_handoff_samples  # noqa: E402


@pytest.mark.parametrize('yaw', [0, 0.1, -0.1])
def test_step_targets_error_in_current_body_axes(yaw):
    e = dict(x=-0.067, y=-0.03, yaw=yaw)
    m = position.plan(e)
    assert math.hypot(m['x'], m['y']) == pytest.approx(0.03)
    nx = e['x'] + math.cos(yaw) * m['x'] - math.sin(yaw) * m['y']
    ny = e['y'] + math.sin(yaw) * m['x'] + math.cos(yaw) * m['y']
    assert math.hypot(nx, ny) == pytest.approx(math.hypot(e['x'], e['y']) - 0.03)
    assert position.plan(dict(x=0.02, y=0.01, yaw=yaw)) is None


@pytest.mark.parametrize(
    'e', [dict(x=0.16, y=0, yaw=0), dict(x=0, y=0, yaw=0.15), dict(x=float('nan'), y=0, yaw=0)]
)
def test_outside_acquisition_never_plans_move(e):
    with pytest.raises(ValueError):
        position.plan(e)


def test_acquisition_range_does_not_change_final_handoff():
    rows = [
        dict(
            time=i * 0.11,
            scan_stamp=i + 1,
            odom_stamp=i + 1,
            odom=dict(x=0, y=0, yaw=0),
            estimate=dict(x=0.07, y=0, yaw=0),
        )
        for i in range(12)
    ]
    assert position._qualify_stopped(rows, 0.15, math.radians(8), 'acquisition')['x'] == 0.07
    with pytest.raises(ValueError, match='5cm/8deg'):
        qualify_handoff_samples(rows)


def test_discrete_steps_remeasure_and_hold_heading(monkeypatch, tmp_path):
    estimates = iter(
        [dict(x=-0.07, y=0, yaw=0), dict(x=-0.04, y=0, yaw=0), dict(x=-0.025, y=0, yaw=0)]
    )
    calls = []

    def collect(*args):
        calls.append('measure')
        return next(estimates)

    monkeypatch.setattr(position, 'collect', collect)

    class Console:
        def __init__(self, *a):
            pass

        def move(self, m):
            assert m.kind == 'move' and m.heading and m.lateral
            calls.append(('move', m.x, m.y))

    monkeypatch.setattr('motion_console.Console', Console)
    r = SimpleNamespace(pose=lambda: Pose(0, 0, 0), moving_pose=lambda: Pose(0, 0, 0), frame='odom')
    position.adjust(r, None, None, io.StringIO(), tmp_path / 'route.json', lambda *a, **k: None)
    assert calls[0::2] == ['measure'] * 3
    assert [x[1] for x in calls if isinstance(x, tuple)] == pytest.approx([0.03, 0.015])
