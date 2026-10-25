"""Pure coarse correction planning; no ROS, browser, or execution authority."""

import copy
import math
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from task8_start_correction import SCOPE, coarse_estimate, plan  # noqa: E402
from task8_p15_localization import BOXES, DIRECTIONS, qualify_samples, register_walls  # noqa: E402


def samples(x=0.08, y=0.0, yaw=0.0):
    return [
        dict(
            time=10 + i * 0.1,
            scan_stamp=1_000_000_000 + i * 100_000_000,
            odom_stamp=1_000_000_000 + i * 100_000_000,
            odom=dict(x=3.0, y=-4.0, yaw=2.0),
            estimate=dict(x=x, y=y, yaw=yaw),
            heading_disagreement=0.0,
        )
        for i in range(12)
    ]


def wall(side, distance, angle=0.0):
    theta = DIRECTIONS[side] + angle
    support = max(abs(math.cos(theta)) * x + abs(math.sin(theta)) * y for x, y in BOXES)
    return dict(gap=distance - support, angle=angle, rms=0.003, span=0.20, count=70, raw_count=75)


def pair(x=0.0, y=0.0, yaw=0.0):
    reference, current = {}, {}
    for side, distance, angle in (('front', 0.58, 0.04), ('right', 0.24, 0.10)):
        reference[side] = wall(side, distance, angle)
        normal = DIRECTIONS[side] + angle
        current[side] = wall(
            side, distance - math.cos(normal) * x - math.sin(normal) * y, angle - yaw
        )
    return reference, current


def test_coarse_eight_cm_can_plan_but_original_registration_stays_strict():
    reference, current = pair(x=0.08)
    result = coarse_estimate(reference, current, dict(x=0.0, y=0.0, yaw=0.0))
    assert result == pytest.approx(dict(x=0.08, y=0.0, yaw=0.0, heading_disagreement=0.0))
    correction = plan(samples(result['x'], result['y'], result['yaw']))
    assert correction['forward_m'] == pytest.approx(-0.03)
    with pytest.raises(ValueError, match='bounded'):
        register_walls(reference, current)
    with pytest.raises(ValueError, match='2cm/1deg'):
        qualify_samples(samples(x=0.08))


@pytest.mark.parametrize('yaw,expected', [(6, -2), (-6, 2), (2, -1), (-2, 1)])
def test_turn_first_half_gain_and_cap(yaw, expected):
    result = plan(samples(x=0.08, y=0.03, yaw=math.radians(yaw)))
    assert result['action'] == 'turn'
    assert result['angle_rad'] == pytest.approx(math.radians(expected))
    assert result['forward_m'] == result['left_m'] == 0
    assert result['scope'] == SCOPE
    assert result['motion_authorized'] is False


def test_translation_uses_current_body_rotation_not_nominal_axes():
    angle = math.radians(0.4)
    result = plan(samples(x=0.01, y=0.012, yaw=angle))
    assert result['action'] == 'translate'
    assert result['forward_m'] == pytest.approx(-math.cos(angle) * 0.01 - math.sin(angle) * 0.012)
    assert result['left_m'] == pytest.approx(math.sin(angle) * 0.01 - math.cos(angle) * 0.012)
    assert result['angle_rad'] == 0


@pytest.mark.parametrize('x,y', [(0.08, 0.04), (-0.08, -0.04), (0.09, -0.06)])
def test_translation_vector_cap_and_direction(x, y):
    result = plan(samples(x=x, y=y))
    assert math.hypot(result['forward_m'], result['left_m']) == pytest.approx(0.03)
    assert result['forward_m'] * x + result['left_m'] * y < 0
    assert result['forward_m'] / result['left_m'] == pytest.approx(x / y)


def test_small_position_waits_for_original_initializer_even_when_yaw_is_acceptable():
    result = plan(samples(x=0.004, y=0.004, yaw=math.radians(0.4)))
    assert result['action'] == 'wait'
    assert result['forward_m'] == result['left_m'] == result['angle_rad'] == 0
    assert result['scope'] == SCOPE
    assert result['motion_authorized'] is False
    assert plan(samples(x=0.008))['action'] == 'wait'
    assert plan(samples(x=0.009))['action'] == 'translate'


@pytest.mark.parametrize(
    'edit',
    [
        lambda s: s.pop(),
        lambda s: s[3].__setitem__('scan_stamp', s[2]['scan_stamp']),
        lambda s: s[3].__setitem__('scan_stamp', s[2]['scan_stamp'] - 1),
        lambda s: s[3].__setitem__('odom_stamp', 0),
        lambda s: s[3].__setitem__('odom_stamp', True),
        lambda s: s[3].__setitem__('time', s[2]['time']),
        lambda s: s[-1].__setitem__('time', s[-2]['time'] + 0.36),
        lambda s: s[-1].__setitem__('scan_stamp', s[-2]['scan_stamp'] + 360_000_000),
        lambda s: [
            row.__setitem__('scan_stamp', 1_000_000_000 + i * 70_000_000) for i, row in enumerate(s)
        ],
        lambda s: s[3].__setitem__('time', float('nan')),
        lambda s: s[3].__setitem__('heading_disagreement', math.radians(1.51)),
        lambda s: s[3].__setitem__('heading_disagreement', float('inf')),
        lambda s: s[3]['odom'].__setitem__('x', 3.006),
        lambda s: s[3]['odom'].__setitem__('yaw', 2.011),
        lambda s: s[3]['estimate'].__setitem__('x', 0.086),
        lambda s: s[3]['estimate'].__setitem__('y', 0.006),
        lambda s: s[3]['estimate'].__setitem__('yaw', math.radians(1.1)),
        lambda s: s[3]['estimate'].__setitem__('x', float('nan')),
        lambda s: s[3]['estimate'].__setitem__('yaw', True),
    ],
)
def test_repeated_discontinuous_unstable_or_invalid_samples_rejected(edit):
    data = samples()
    edit(data)
    with pytest.raises(ValueError):
        plan(data)


@pytest.mark.parametrize('x,y,yaw', [(0.151, 0, 0), (0.12, 0.10, 0), (0, 0, math.radians(8.01))])
def test_coarse_region_is_bounded_for_every_sample(x, y, yaw):
    with pytest.raises(ValueError, match='15cm/8deg'):
        plan(samples(x=x, y=y, yaw=yaw))


def test_group_stopped_gate_is_pairwise_not_only_relative_to_first():
    data = samples()
    data[1]['odom']['x'] -= 0.004
    data[2]['odom']['x'] += 0.004
    with pytest.raises(ValueError, match='5 mm'):
        plan(data)


@pytest.mark.parametrize(
    'field,value', [('angle', math.radians(12.1)), ('count', 59), ('rms', 0.013), ('span', 0.079)]
)
def test_coarse_estimator_preserves_wall_quality_gates(field, value):
    reference, current = pair(x=0.08)
    current['front'][field] = value
    with pytest.raises(ValueError):
        coarse_estimate(reference, current, dict(x=0, y=0, yaw=0))


def test_coarse_estimator_rejects_disagreement_parallel_and_out_of_region():
    reference, current = pair()
    bad = copy.deepcopy(current)
    bad['right']['angle'] += math.radians(2)
    with pytest.raises(ValueError, match='disagree'):
        coarse_estimate(reference, bad, dict(x=0, y=0, yaw=0))
    reference['rear'] = current['rear'] = wall('rear', 0.5)
    with pytest.raises(ValueError, match='parallel'):
        coarse_estimate(reference, current, dict(x=0, y=0, yaw=0), ('front', 'rear'))
    with pytest.raises(ValueError, match='15cm/8deg'):
        coarse_estimate(reference, current, dict(x=0.151, y=0, yaw=0))


@pytest.mark.parametrize(
    'field,value', [('count', float('inf')), ('raw_count', float('nan')), ('count', True)]
)
def test_nonfinite_or_boolean_wall_counts_fail(field, value):
    reference, current = pair()
    current['front'][field] = value
    with pytest.raises(ValueError):
        coarse_estimate(reference, current, dict(x=0, y=0, yaw=0))


@pytest.mark.parametrize('data', [None, {}, [None] * 12, [{}] * 12])
def test_malformed_sample_shape_fails_closed(data):
    with pytest.raises(ValueError):
        plan(data)
