"""Synthetic raw scans and explicit fixed-side observation contracts; no robot."""

import copy
import math
from pathlib import Path
import sys
from unittest.mock import patch
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from task8_runner import backend_imports

backend_imports()
from task8_wall_observation import observe, require_patch  # noqa: E402
from task8_front_side import estimate  # noqa: E402


def cloud():
    values = []
    for i in range(81):
        y = -0.2 + i * 0.005
        x = 0.6 - math.tan(0.01) * y
        values.append((x, y, math.atan2(y, x)))
        x = -0.08 + i * 0.002
        y = -0.26 + math.tan(0.08) * x
        values.append((x, y, math.atan2(y, x)))
    return values, dict(front=81, right=81, right_wide=81)


def test_raw_slanted_side_with_independent_front_heading():
    points, counts = cloud()
    reference = observe(points, counts)
    x, y, yaw = 0.012, -0.009, 0.018
    moved = [
        (
            math.cos(yaw) * (u - x) + math.sin(yaw) * (v - y),
            -math.sin(yaw) * (u - x) + math.cos(yaw) * (v - y),
        )
        for u, v, _ in points
    ]
    current = observe([(u, v, math.atan2(v, u)) for u, v in moved], counts, reference)
    actual = estimate(reference, current, dict(x=0, y=0, yaw=0))
    assert actual['x'] == pytest.approx(x, abs=0.001)
    assert actual['y'] == pytest.approx(y, abs=0.001)
    assert actual['yaw'] == pytest.approx(yaw, abs=0.001)
    require_patch(reference, current, actual)
    assert current['_policy'] == 'front_heading_fixed_reference_side'
    assert '_right_tls' in current


def test_sector_counts_are_actual_twenty_degree_sector():
    points, counts = cloud()
    with patch(
        'wall_geometry.side_distance', wraps=__import__('wall_geometry').side_distance
    ) as fn:
        observe(points, counts)
    args = fn.call_args.args
    assert args[1]['right_wide'] == counts['right']
    assert all(
        abs(math.remainder(p[2] + math.pi / 2, 2 * math.pi)) <= math.radians(20) for p in args[0]
    )


@pytest.mark.parametrize('shift', [0.06, 0.2])
def test_patch_shift_is_rejected(shift):
    points, counts = cloud()
    reference = observe(points, counts)
    current = copy.deepcopy(reference)
    for k in ('low', 'high', 'center'):
        current['_patch'][k] += shift
    with pytest.raises(ValueError, match='correspondence'):
        require_patch(reference, current, dict(x=0, y=0, yaw=0))


def test_front_short_span_rejected():
    points, counts = cloud()
    from wall_geometry import fit_wall

    wall = fit_wall(points, counts, 'front')
    from dataclasses import replace

    with patch('wall_geometry.fit_wall', return_value=replace(wall, span=0.29)):
        with pytest.raises(ValueError, match='long-front'):
            observe(points, counts)


def test_current_unused_tls_failure_does_not_replace_constrained_distance_quality():
    points, counts = cloud()
    reference = observe(points, counts)
    from wall_geometry import fit_wall

    def diagnostic_missing(p, c, side):
        if side == 'right':
            raise ValueError('Diagnostic TLS unavailable')
        return fit_wall(p, c, side)

    with patch('wall_geometry.fit_wall', side_effect=diagnostic_missing):
        current = observe(points, counts, reference)
    assert 'unavailable' in current['_right_tls']
    assert current['right']['count'] * 5 >= current['right']['raw_count'] * 4
    actual = estimate(reference, current, dict(x=0, y=0, yaw=0))
    require_patch(reference, current, actual)
