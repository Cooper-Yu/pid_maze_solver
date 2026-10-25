"""Pure servo sign, bounds, freshness, and readiness tests; no ROS or execution."""

import math
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from task8_wall_servo import velocity  # noqa: E402


def command(x=0.0, y=0.0, yaw=0.0, **kwargs):
    return velocity(
        dict(x=x, y=y, yaw=yaw),
        kwargs.get('drift', 0.0),
        kwargs.get('elapsed', 0.0),
        kwargs.get('source_age', 0.0),
        kwargs.get('receipt_age', 0.0),
    )


def test_zero_pose_ready_and_zero_command():
    assert command() == ((0.0, 0.0, 0.0), True)
    assert command(x=0.003, y=0.004, yaw=math.radians(0.3)) == ((0.0, 0.0, 0.0), True)


@pytest.mark.parametrize('yaw', [math.radians(0.8) + 1e-10, math.radians(2), math.radians(-2)])
def test_heading_first_disables_translation(yaw):
    (vx, vy, wz), ready = command(x=0.03, y=0.02, yaw=yaw)
    assert vx == vy == 0.0
    assert wz * yaw < 0
    assert abs(wz) <= 0.02
    assert ready is False


def test_exact_heading_boundary_allows_translation_and_heading_feedback():
    (vx, vy, wz), ready = command(x=0.01, yaw=math.radians(0.8))
    assert math.hypot(vx, vy) > 0
    assert wz < 0
    assert ready is False


@pytest.mark.parametrize('yaw', [0.01, -0.01, math.radians(0.31), math.radians(-0.31)])
def test_no_console_point_zero_one_radian_heading_deadband(yaw):
    (vx, vy, wz), ready = command(yaw=yaw)
    assert vx == vy == 0
    assert wz == pytest.approx(-0.5 * yaw)
    assert wz != 0
    assert ready is False


@pytest.mark.parametrize('yaw', [math.radians(0.3), 0.0, math.radians(-0.3)])
def test_heading_ready_band_has_no_angular_command(yaw):
    (vx, vy, wz), ready = command(yaw=yaw)
    assert vx == vy == wz == 0
    assert ready is True


@pytest.mark.parametrize(
    'x,y,yaw', [(0.006, 0.004, 0.01), (-0.006, -0.004, -0.01), (0.005, -0.008, 0.006)]
)
def test_body_rotation_transform_and_half_gain(x, y, yaw):
    (vx, vy, wz), ready = command(x=x, y=y, yaw=yaw)
    assert vx == pytest.approx(-0.5 * (math.cos(yaw) * x + math.sin(yaw) * y))
    assert vy == pytest.approx(0.5 * (math.sin(yaw) * x - math.cos(yaw) * y))
    # Transform the command back to nominal axes: it must oppose displacement.
    nx, ny = math.cos(yaw) * vx - math.sin(yaw) * vy, math.sin(yaw) * vx + math.cos(yaw) * vy
    assert nx == pytest.approx(-0.5 * x)
    assert ny == pytest.approx(-0.5 * y)
    assert ready is False


@pytest.mark.parametrize('x,y', [(0.03, 0.04), (-0.03, 0.04), (0.05, 0.0), (0.0, -0.05)])
def test_translation_norm_speed_cap_not_axiswise_cap(x, y):
    (vx, vy, wz), ready = command(x=x, y=y)
    assert math.hypot(vx, vy) == pytest.approx(0.01)
    assert wz == 0
    assert ready is False


@pytest.mark.parametrize('yaw', [math.radians(8), math.radians(-8)])
def test_angular_speed_cap(yaw):
    (_, _, wz), ready = command(yaw=yaw)
    assert abs(wz) == 0.02
    assert wz * yaw < 0
    assert ready is False


def test_position_boundary():
    assert command(x=0.005)[1] is True
    result, ready = command(x=0.0050001)
    assert result[0] < 0
    assert ready is False


@pytest.mark.parametrize(
    'kwargs',
    [
        dict(drift=0.04, elapsed=30, source_age=-0.1, receipt_age=0.30),
        dict(drift=0, elapsed=0, source_age=0.30, receipt_age=0),
    ],
)
def test_inclusive_budget_and_age_boundaries(kwargs):
    assert command(**kwargs)[1] is True


@pytest.mark.parametrize(
    'kwargs',
    [
        dict(drift=0.0400001),
        dict(drift=-0.0001),
        dict(elapsed=30.0001),
        dict(elapsed=-0.1),
        dict(source_age=0.300001),
        dict(source_age=-0.100001),
        dict(receipt_age=0.300001),
        dict(receipt_age=-0.0001),
        dict(x=0.050001),
        dict(x=0.04, y=0.04),
        dict(yaw=math.radians(8.00001)),
    ],
)
def test_invalid_or_out_of_budget_data_raise(kwargs):
    with pytest.raises(ValueError):
        command(**kwargs)


@pytest.mark.parametrize(
    'field', ['x', 'y', 'yaw', 'drift', 'elapsed', 'source_age', 'receipt_age']
)
@pytest.mark.parametrize('bad', [float('nan'), float('inf'), float('-inf'), True])
def test_every_numeric_input_rejects_nonfinite_or_boolean(field, bad):
    with pytest.raises(ValueError):
        command(**{field: bad})


@pytest.mark.parametrize('estimate', [None, {}, dict(x=0, y=0)])
def test_missing_pose_raises(estimate):
    with pytest.raises(ValueError):
        velocity(estimate, 0.0, 0.0, 0.0, 0.0)
