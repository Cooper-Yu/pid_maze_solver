"""Geometric counterexamples for the bounded P15 registration contract."""

import math
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from task8_p15_localization import (  # noqa: E402
    BOXES,
    DIRECTIONS,
    compose,
    nominal_estimate,
    origin_from_current,
    register_walls,
)


def wall(side, distance, angle=0):
    a = DIRECTIONS[side] + angle
    support = max(abs(math.cos(a)) * x + abs(math.sin(a)) * y for x, y in BOXES)
    return dict(gap=distance - support, angle=angle, rms=0.003, span=0.2, count=70, raw_count=75)


def pair(x=0, y=0, yaw=0):
    reference = {s: wall(s, d, a) for s, d, a in (('front', 0.58, 0.04), ('right', 0.24, 0.10))}
    current = {}
    for s, d, a in (('front', 0.58, 0.04), ('right', 0.24, 0.10)):
        n = DIRECTIONS[s] + a
        current[s] = wall(s, d - math.cos(n) * x - math.sin(n) * y, a - yaw)
    return reference, current


@pytest.mark.parametrize('x,y,yaw', [(0, 0, 0), (0.012, -0.008, 0.01), (-0.02, 0.015, -0.02)])
def test_two_lines_recover_known_pose(x, y, yaw):
    reference, current = pair(x, y, yaw)
    assert register_walls(reference, current) == pytest.approx(dict(x=x, y=y, yaw=yaw))


def test_actual_terminal_error_is_preserved():
    reference, current = pair()
    source = dict(x=-0.0126, y=-0.00845, yaw=0.00496)
    assert nominal_estimate(reference, current, source) == pytest.approx(source)


@pytest.mark.parametrize(
    'new_odom',
    [
        dict(x=0, y=0, yaw=0),
        dict(x=50, y=-90, yaw=1.7),
        dict(x=-1, y=2, yaw=-3.1),
    ],
)
def test_new_epoch_origin_reconstructs_current_pose(new_odom):
    local = dict(x=0.012, y=-0.008, yaw=0.01)
    origin = origin_from_current(new_odom, local)
    reconstructed = compose(origin, local)
    assert reconstructed == pytest.approx(new_odom)


def test_inconsistent_wall_heading_rejected():
    reference, current = pair()
    current['right']['angle'] += math.radians(2)
    with pytest.raises(ValueError, match='disagree'):
        register_walls(reference, current)


def test_parallel_reference_walls_rejected():
    reference, current = pair()
    reference['rear'] = wall('rear', 0.4)
    current['rear'] = wall('rear', 0.4)
    with pytest.raises(ValueError, match='parallel'):
        register_walls(reference, current, ('front', 'rear'))


@pytest.mark.parametrize(
    'field,value',
    [
        ('gap', -0.001),
        ('gap', float('nan')),
        ('rms', 0.02),
        ('span', 0.02),
        ('count', 4),
        ('angle', True),
        ('angle', math.radians(15)),
    ],
)
def test_bad_fit_rejected(field, value):
    reference, current = pair()
    current['front'][field] = value
    with pytest.raises(ValueError):
        register_walls(reference, current)


@pytest.mark.parametrize('x,y,yaw', [(0.051, 0, 0), (0, 0, math.radians(6))])
def test_outside_local_region_rejected(x, y, yaw):
    reference, current = pair(x, y, yaw)
    with pytest.raises(ValueError, match='Outside bounded'):
        register_walls(reference, current)
