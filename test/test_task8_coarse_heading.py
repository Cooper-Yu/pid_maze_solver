"""Coarse acquisition uses real scheduled rays and bounded heading-only actions."""

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
from task8_coarse_heading import observe, qualify, fit_side  # noqa: E402
import task8_coarse_heading_runner as runtime  # noqa: E402
from wall_geometry import fit_wall  # noqa: E402


def corridor(yaw):
    points = []
    counts = dict(left=0, right=0, front=0, rear=0)
    for i in range(720):
        a = -math.pi + i * 2 * math.pi / 720
        for side, center in [
            ('left', math.pi / 2),
            ('right', -math.pi / 2),
            ('front', 0),
            ('rear', math.pi),
        ]:
            if abs(math.remainder(a - center, 2 * math.pi)) <= math.radians(20):
                counts[side] += 1
        c, s = math.cos(a + yaw), math.sin(a + yaw)
        distances = []
        if abs(c) > 1e-9:
            distances.append((0.60 if c > 0 else -0.50) / c)
        if abs(s) > 1e-9:
            distances.append((0.28 if s > 0 else -0.26) / s)
        d = min(distances)
        points.append((d * math.cos(a), d * math.sin(a), a))
    return points, counts


@pytest.mark.parametrize('degrees', [-24, -19, 0, 19, 24])
def test_large_heading_has_correct_turn_sign_without_loosening_regular_fit(degrees):
    p, c = corridor(math.radians(degrees))
    oldp, oldc = corridor(0)
    reference = {'right': fit_side(oldp, oldc, 'right')}
    result = observe(p, c, reference, 0)
    assert result['correction'] == pytest.approx(-math.radians(degrees), abs=1e-8)
    if abs(degrees) > 12:
        with pytest.raises(ValueError):
            fit_wall(p, c, 'right')


def test_missing_rays_still_count_against_coverage():
    p, c = corridor(0)
    sparse = [v for i, v in enumerate(p) if i % 3 == 0]
    with pytest.raises(ValueError):
        fit_side(sparse, c, 'right')


def test_disagreeing_sides_and_large_correction_are_rejected(monkeypatch):
    wall = dict(angle=0.3, gap=0.1, rms=0.002, span=0.2, count=60, raw_count=60)
    monkeypatch.setattr(
        'task8_coarse_heading.fit_side',
        lambda p, c, s: dict(wall, angle=0.3 if s == 'right' else 0.5),
    )
    with pytest.raises(ValueError, match='disagree'):
        observe([], {}, {'right': wall}, 0)
    monkeypatch.setattr('task8_coarse_heading.fit_side', lambda *a: dict(wall))
    with pytest.raises(ValueError, match='25deg'):
        observe([], {}, {'right': dict(wall, angle=-0.2)}, 0)


def batch():
    return [
        dict(
            time=i * 0.11,
            scan_stamp=i + 1,
            odom_stamp=i + 1,
            odom=dict(x=0.0, y=0.0, yaw=0.0),
            correction=0.2,
        )
        for i in range(12)
    ]


@pytest.mark.parametrize('fault', ['replay', 'moving', 'stale', 'unstable', 'nan'])
def test_acquisition_rejects_invalid_batches(fault):
    rows = batch()
    if fault == 'replay':
        rows[-1]['scan_stamp'] = rows[-2]['scan_stamp']
    if fault == 'moving':
        rows[-1]['odom']['x'] = 0.006
    if fault == 'stale':
        rows[-1]['time'] += 0.4
    if fault == 'unstable':
        rows[-1]['correction'] += 0.05
    if fault == 'nan':
        rows[-1]['correction'] = float('nan')
    with pytest.raises(ValueError):
        qualify(rows)


def test_batch_and_source_residual_not_rebased():
    assert qualify(batch()) == pytest.approx(0.2)
    p, c = corridor(-0.2)
    oldp, oldc = corridor(0)
    r = {'right': fit_side(oldp, oldc, 'right')}
    assert observe(p, c, r, 0.01)['correction'] == pytest.approx(0.19)


def test_align_uses_only_existing_turn_pid_then_remeasures(monkeypatch, tmp_path):
    poses = [Pose(0, 0, 0)]
    runner = SimpleNamespace(
        pose=lambda: poses[-1], moving_pose=lambda: poses[-1], frame='odom', geometry=lambda: None
    )
    moves = []
    events = []
    proposals = iter([0.30, 0.04, 0.005])
    monkeypatch.setattr(runtime, 'collect', lambda *a: dict(correction=next(proposals)))

    class Console:
        def __init__(self, *args):
            self.guard = args[-1]

        def move(self, motion):
            assert self.guard((0, 0, 0.1)) == (0, 0, 0.1)
            assert motion.kind == 'turn'
            moves.append(motion.angle)

    monkeypatch.setattr('motion_console.Console', Console)
    runtime.align(
        runner,
        None,
        None,
        io.StringIO(),
        tmp_path / 'route.json',
        lambda event, **data: events.append((event, data)),
    )
    assert moves == pytest.approx([math.radians(15), 0.04])
    assert events[-1][0] == 'p15_coarse_heading_handoff'
    assert events[-1][1]['position_verified'] is False


def test_heading_only_stage_retains_overall_four_cm_limit(monkeypatch, tmp_path):
    anchor = Pose(0, 0, 0)
    runner = SimpleNamespace(
        pose=lambda: anchor, moving_pose=lambda: Pose(0.041, 0, 0), frame='odom'
    )

    def collect(*args):
        args[3]()
        return dict(correction=0.2)

    class Console:
        def __init__(self, *args):
            self.guard = args[-1]

        def move(self, motion):
            self.guard((0, 0, 0.1))

    monkeypatch.setattr('motion_console.Console', Console)
    monkeypatch.setattr(runtime, 'collect', collect)
    with pytest.raises(RuntimeError, match='4cm'):
        runtime.align(
            runner, None, None, io.StringIO(), tmp_path / 'route.json', lambda *a, **k: None
        )


@pytest.mark.parametrize(
    'invalid_policy,align_failure', [(False, False), (True, False), (False, True)]
)
def test_session_checks_policy_before_align_and_localizes_afterward(
    monkeypatch, tmp_path, invalid_policy, align_failure
):
    from unittest.mock import patch
    import task8_runner
    import task8_p15_start

    order = []
    runner = SimpleNamespace(exclusive=lambda: None, frame='odom')
    monkeypatch.setattr('wall_runtime.WallRunner', lambda *a, **k: runner)
    monkeypatch.setattr(
        'route_feedback_pause.RouteFeedbackPause', lambda *a: SimpleNamespace(check=lambda: None)
    )
    monkeypatch.setattr('task6_route_runner.close_runner', lambda *a: order.append('closed'))
    monkeypatch.setattr(task8_p15_start, 'validate_source', lambda *a: {})
    monkeypatch.setattr(task8_p15_start, 'load_reference', lambda *a: ({}, None, None, None))

    def policy(*args):
        order.append('policy')
        if invalid_policy:
            raise ValueError('policy')

    monkeypatch.setattr(task8_p15_start, 'start_qualifier', policy)

    def align(*args):
        order.append('align')
        if align_failure:
            raise RuntimeError('coarse acquisition failed')

    monkeypatch.setattr(runtime, 'align', align)

    def initialize(*args):
        order.append('initialize')
        if order.count('initialize') == 1:
            raise RuntimeError('P15 initialization wait expired: wall unavailable')
        raise RuntimeError('final initialization still required')

    args = SimpleNamespace(
        speed=0.06,
        turn_speed=0.2,
        max_seconds=600,
        p15_start='localized',
        p15_observation='front_heading_fixed_reference_side',
        source_book=Path('/book'),
        source_bag=Path('/bag'),
        endpoint=(0, 0),
    )
    with patch.object(task8_runner, 'run_actions') as run:
        with pytest.raises((RuntimeError, ValueError)):
            task8_runner.session(args, tmp_path / 'route.json', io.StringIO(), [], initialize)
        run.assert_not_called()
    expected = ['policy'] + ([] if invalid_policy else ['initialize', 'align'])
    if not invalid_policy and not align_failure:
        expected.append('initialize')
    assert order == expected + ['closed']
