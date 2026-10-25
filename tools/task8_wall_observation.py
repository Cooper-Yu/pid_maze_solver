"""Explicit front-heading / fixed-reference-side observations, not two yaw sensors."""

from dataclasses import asdict
import math
from statistics import median


def observe(points, counts, reference=None):
    from wall_geometry import fit_wall, side_distance

    front = fit_wall(points, counts, 'front')
    if front.span < 0.30 or front.rms > 0.008:
        raise ValueError('P15 long-front heading observation unavailable')
    try:
        diagnostic = asdict(fit_wall(points, counts, 'right'))
    except ValueError as error:
        if reference is None:
            raise
        diagnostic = dict(unavailable=str(error))
    theta = 0.0 if reference is None else reference['front']['angle'] - front.angle
    angle = diagnostic['angle'] if reference is None else reference['right']['angle'] - theta
    selected = [
        p
        for p in points
        if abs(math.remainder(p[2] + math.pi / 2, 2 * math.pi)) <= math.radians(20)
    ]
    expected = dict(counts)
    expected['right_wide'] = counts['right']
    right = side_distance(selected, expected, 'right', angle)
    # Track the same bounded patch; this is a correspondence diagnostic, not proof
    # against all possible visually similar walls.
    nx, ny = math.cos(-math.pi / 2 + angle), math.sin(-math.pi / 2 + angle)
    offsets = [nx * x + ny * y for x, y, _ in selected]
    center = median(offsets)
    patch = [ny * x - nx * y for x, y, _ in selected if abs(nx * x + ny * y - center) <= 0.020]
    result = dict(
        front=asdict(front),
        right=asdict(right),
        _policy='front_heading_fixed_reference_side',
        _patch=dict(low=min(patch), high=max(patch), center=median(patch)),
        _right_tls=diagnostic,
    )
    return result


def require_patch(reference, current, relative):
    """Reject different visible side-wall sections before using a fixed normal."""
    from task8_p15_localization import normal_wall

    nx, ny, _, _ = normal_wall('right', reference['right'])
    shift = ny * relative['x'] - nx * relative['y']
    a, b = reference['_patch'], current['_patch']
    low, high = b['low'] + shift, b['high'] + shift
    overlap = max(0.0, min(a['high'], high) - max(a['low'], low))
    shorter = min(a['high'] - a['low'], high - low)
    if shorter <= 0 or overlap / shorter < 0.75 or abs(b['center'] + shift - a['center']) > 0.05:
        raise ValueError('P15 side wall patch correspondence unavailable')
