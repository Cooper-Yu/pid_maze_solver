"""No-motion geometric contracts for the explicit fixed-source side-normal model."""

import copy
import math
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from task8_front_side import MODEL, estimate  # noqa: E402
from task8_p15_localization import BOXES, DIRECTIONS, compose, qualify_samples  # noqa: E402


def wall(side, distance, angle):
    theta = DIRECTIONS[side] + angle
    support = max(abs(math.cos(theta)) * x + abs(math.sin(theta)) * y for x, y in BOXES)
    return dict(
        gap=distance - support,
        angle=angle,
        rms=0.005,
        span=0.40 if side == 'front' else 0.18,
        count=70,
        raw_count=75,
    )


def observations(x=0.0, y=0.0, yaw=0.0, right_angle=0.10):
    reference, current = {}, {}
    for side, distance, angle in (('front', 0.58, 0.026), ('right', 0.25, right_angle)):
        normal = DIRECTIONS[side] + angle
        reference[side] = wall(side, distance, angle)
        current[side] = wall(
            side, distance - math.cos(normal) * x - math.sin(normal) * y, angle - yaw
        )
    return reference, current


@pytest.mark.parametrize(
    'x,y,yaw', [(0.012, -0.008, 0.01), (-0.021, 0.025, -0.02), (0.025, 0.025, 0.04), (0, 0, 0)]
)
def test_slanted_side_normal_recovers_translation_and_front_heading(x, y, yaw):
    reference, current = observations(x, y, yaw)
    assert estimate(reference, current, dict(x=0.0, y=0.0, yaw=0.0)) == pytest.approx(
        dict(x=x, y=y, yaw=yaw)
    )
    assert MODEL == 'front_heading_fixed_reference_side'


@pytest.mark.parametrize(
    'source', [dict(x=-0.01261, y=-0.00845, yaw=0.004957), dict(x=0.01, y=0.02, yaw=-0.2)]
)
def test_arbitrary_source_residual_is_composed_not_erased(source):
    reference, current = observations(0.01, -0.007, 0.02)
    expected = compose(source, dict(x=0.01, y=-0.007, yaw=0.02))
    assert estimate(reference, current, source) == pytest.approx(expected)


def test_body_support_is_rotated_and_not_subtracted_twice():
    reference, current = observations(0.02, 0.01, 0.04, right_angle=0.18)
    assert reference['right']['gap'] != current['right']['gap']
    result = estimate(reference, current, dict(x=0.0, y=0.0, yaw=0.0))
    assert result['x'] == pytest.approx(0.02)
    assert result['y'] == pytest.approx(0.01)


@pytest.mark.parametrize('mismatch', [1.1e-8, 0.001, -0.001])
def test_independent_or_wrong_current_side_angle_cannot_masquerade_as_fixed_normal(mismatch):
    reference, current = observations(yaw=0.01)
    current['right']['angle'] += mismatch
    with pytest.raises(ValueError, match='transported fixed source normal'):
        estimate(reference, current, dict(x=0, y=0, yaw=0))


def test_front_normal_cannot_replace_slanted_reference_side_normal():
    reference, current = observations(yaw=0.01)
    current['right']['angle'] = current['front']['angle']
    with pytest.raises(ValueError, match='transported fixed source normal'):
        estimate(reference, current, dict(x=0, y=0, yaw=0))


@pytest.mark.parametrize(
    'which,side,field,value',
    [
        ('source', 'front', 'span', 0.299),
        ('current', 'front', 'span', 0.299),
        ('source', 'front', 'rms', 0.00801),
        ('current', 'front', 'rms', 0.00801),
        ('current', 'right', 'span', 0.079),
        ('source', 'right', 'rms', 0.01201),
        ('current', 'right', 'count', 59),
        ('source', 'right', 'count', 7),
        ('source', 'front', 'angle', math.radians(12.01)),
        ('source', 'right', 'gap', 0.0),
        ('current', 'right', 'angle', float('nan')),
        ('current', 'right', 'count', float('inf')),
        ('source', 'front', 'rms', float('nan')),
    ],
)
def test_original_wall_quality_and_stronger_long_front_gates(which, side, field, value):
    reference, current = observations()
    target = reference if which == 'source' else current
    target[side][field] = value
    with pytest.raises(ValueError):
        estimate(reference, current, dict(x=0, y=0, yaw=0))


@pytest.mark.parametrize('x,y,yaw', [(0.0501, 0, 0), (0.04, 0.04, 0), (0, 0, math.radians(5.01))])
def test_original_registration_region_remains_five_cm_five_degrees(x, y, yaw):
    reference, current = observations(x, y, yaw)
    with pytest.raises(ValueError, match='5cm/5deg'):
        estimate(reference, current, dict(x=0, y=0, yaw=0))


def sample_batch(value):
    return [
        dict(
            time=1 + i * 0.1,
            scan_stamp=1_000_000_000 + i * 100_000_000,
            odom_stamp=1_000_000_000 + i * 100_000_000,
            odom=dict(x=1, y=2, yaw=0),
            estimate=copy.deepcopy(value),
        )
        for i in range(12)
    ]


def test_original_final_two_cm_one_degree_gate_is_not_bypassed():
    reference, current = observations(0.025, 0, 0)
    value = estimate(reference, current, dict(x=0, y=0, yaw=0))
    with pytest.raises(ValueError, match='2cm/1deg'):
        qualify_samples(sample_batch(value))
    reference, current = observations(0, 0, math.radians(1.1))
    value = estimate(reference, current, dict(x=0, y=0, yaw=0))
    with pytest.raises(ValueError, match='2cm/1deg'):
        qualify_samples(sample_batch(value))
    reference, current = observations(0.005, 0.005, math.radians(0.3))
    value = estimate(reference, current, dict(x=-0.005, y=0.002, yaw=0.001))
    assert qualify_samples(sample_batch(value)) == pytest.approx(value)
