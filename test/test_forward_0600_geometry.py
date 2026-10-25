"""Approved forward changes: metric endpoints, straight legs and preserved surroundings."""

import math
from pathlib import Path
import sys
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
from scene_route import load_scene  # noqa: E402


def test_approved_forward_legs_and_shifted_p13():
    d = load_scene(ROOT / 'waypoints/waypoints_real.yaml', 2)
    a = {x['name']: x for x in d['actions']}
    assert a['P11']['points'][-1] == pytest.approx([0.8518674948445645, -1.8579827169427832])
    assert a['P12']['points'][-1] == pytest.approx([0.8518674948445645, -1.0615868730786444])
    assert a['P13']['points'][-1] == pytest.approx([0.3458019547712687, -1.0315868730786444])
    start, end = a['P13']['points'][0], a['P13']['points'][-1]
    dx, dy = end[0] - start[0], end[1] - start[1]
    assert dy == pytest.approx(0.03)
    assert all(
        abs((p[0] - start[0]) * dy - (p[1] - start[1]) * dx) < 1e-12 for p in a['P13']['points']
    )
    assert all(p[0] == a['P12']['points'][0][0] for p in a['P12']['points'])
    paths = [x for x in d['actions'] if x['kind'] == 'path']
    for x, y in zip(paths, paths[1:]):
        assert x['points'][-1] == pytest.approx(y['points'][0])
    assert all(
        math.dist(p, q) <= 0.05 + 1e-9 for x in paths for p, q in zip(x['points'], x['points'][1:])
    )
    assert len(d['actions']) == 21
    assert d['actions'][-1]['name'] == 'P15_after_turn'
