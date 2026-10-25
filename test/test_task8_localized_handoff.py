"""Measured offset handoff keeps nominal origin and all later route goals."""

import math
from pathlib import Path
import sys
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from task8_runner import backend_imports, localized_first_edge

backend_imports()
from action_plan import Pose  # noqa: E402
from motion_model import Motion  # noqa: E402
from task8_p15_localization import (  # noqa: E402
    qualify_samples,
    qualify_handoff_samples,
    origin_from_current,
    compose,
)  # noqa: E402
from task8_p15_start import start_qualifier  # noqa: E402


def samples():
    return [
        dict(
            time=i * 0.11,
            scan_stamp=i + 1,
            odom_stamp=i + 1,
            odom=dict(x=20.0, y=30.0, yaw=1.5),
            estimate=dict(x=-0.015, y=-0.018, yaw=math.radians(5)),
        )
        for i in range(12)
    ]


def test_handoff_accepts_offset_but_precise_still_rejects():
    s = samples()
    with pytest.raises(ValueError, match='2cm/1deg'):
        qualify_samples(s)
    estimate = qualify_handoff_samples(s)
    origin = origin_from_current(s[-1]['odom'], estimate)
    assert compose(origin, estimate) == pytest.approx(s[-1]['odom'])
    assert origin != s[-1]['odom']


@pytest.mark.parametrize('kind', ['repeated', 'stale', 'moving', 'outside', 'yaw', 'nonfinite'])
def test_handoff_keeps_freshness_and_bounds(kind):
    s = samples()
    if kind == 'repeated':
        s[-1]['scan_stamp'] = s[-2]['scan_stamp']
    if kind == 'stale':
        s[-1]['time'] += 0.5
    if kind == 'moving':
        s[-1]['odom']['x'] += 0.006
    if kind == 'outside':
        s[-1]['estimate']['x'] = 0.051
    if kind == 'yaw':
        s[-1]['estimate']['yaw'] = math.radians(8.1)
    if kind == 'nonfinite':
        s[-1]['estimate']['x'] = math.nan
    with pytest.raises(ValueError):
        qualify_handoff_samples(s)


def test_policy_must_be_explicit_and_reference_is_not_rebased():
    ref = ({}, None, None, None)
    with pytest.raises(ValueError):
        start_qualifier(ref, 'localized', 'two_wall_tls')
    fn = start_qualifier(ref, 'localized', 'front_heading_fixed_reference_side')
    assert fn is qualify_handoff_samples
    assert ref[0]['_localized_handoff'] is True


def test_first_edge_keeps_final_waypoints_and_fixed_heading():
    motion = Motion('path', angle=0.0, points=((0.0, 0.0), (0.2, 0.0), (0.4, 0.1)))
    corrected = localized_first_edge(
        motion, Pose(0.0, 0.0, 0.0), Pose(-0.015, -0.018, math.radians(5))
    )
    assert corrected.points[0] == pytest.approx((-0.015, -0.018))
    assert corrected.points[-2:] == motion.points[-2:]
    assert corrected.angle == 0.0
    assert all(
        math.dist(a, b) <= 0.05 + 1e-9
        for a, b in zip(corrected.points[:-2], corrected.points[1:-1])
    )
    with pytest.raises(RuntimeError):
        localized_first_edge(motion, Pose(0.0, 0.0, 0.0), Pose(0.051, 0.0, 0.0))
