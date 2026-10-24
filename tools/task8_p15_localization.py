"""Bounded P15 two-wall registration, independent of the previous odometry epoch.

This is a local pose estimate near a visually confirmed P15, not global localization.
No ROS imports, file access, control publishers, or motion occur in this module.
"""

import math

DIRECTIONS = {'front': 0.0, 'left': math.pi / 2, 'right': -math.pi / 2, 'rear': math.pi}
BOXES = ((0.170, 0.135), (0.135, 0.160))


def finite(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError('Expected finite number')
    return value


def normal_wall(side, wall):
    """Recover normal and origin distance from the existing body-clearance wall fit."""
    angle = finite(wall['angle'])
    gap = finite(wall['gap'])
    rms = finite(wall['rms'])
    span = finite(wall['span'])
    if abs(angle) > math.radians(12) or gap <= 0 or not 0 <= rms <= 0.012 or span < 0.08:
        raise ValueError('Unreliable P15 wall fit')
    if wall['count'] < 8 or wall['count'] * 5 < wall['raw_count'] * 4:
        raise ValueError('Insufficient P15 wall consensus')
    direction = DIRECTIONS[side] + angle
    nx, ny = math.cos(direction), math.sin(direction)
    offset = gap + max(abs(nx) * x + abs(ny) * y for x, y in BOXES)
    return nx, ny, offset, angle


def register_walls(reference, current, sides=('front', 'right')):
    """Estimate current body pose in reference body frame from two nonparallel lines."""
    if len(sides) != 2 or len(set(sides)) != 2:
        raise ValueError('Two distinct wall directions required')
    previous, actual = [], []
    for side in sides:
        if side not in DIRECTIONS:
            raise ValueError('Unknown wall side')
        previous.append(normal_wall(side, reference[side]))
        actual.append(normal_wall(side, current[side]))
    rotations = [math.remainder(a[3] - b[3], 2 * math.pi) for a, b in zip(previous, actual)]
    if abs(math.remainder(rotations[0] - rotations[1], 2 * math.pi)) > math.radians(1.5):
        raise ValueError('P15 walls disagree on heading')
    theta = math.atan2(sum(math.sin(x) for x in rotations), sum(math.cos(x) for x in rotations))
    a, b = previous
    determinant = a[0] * b[1] - a[1] * b[0]
    if abs(determinant) < 0.7:
        raise ValueError('P15 wall directions are too parallel')
    d0, d1 = a[2] - actual[0][2], b[2] - actual[1][2]
    x = (d0 * b[1] - a[1] * d1) / determinant
    y = (a[0] * d1 - d0 * b[0]) / determinant
    if math.hypot(x, y) > 0.05 or abs(theta) > math.radians(5):
        raise ValueError('Outside bounded P15 registration region')
    return {'x': x, 'y': y, 'yaw': theta}


def compose(first, second):
    x, y, a = (finite(first[k]) for k in ('x', 'y', 'yaw'))
    u, v, b = (finite(second[k]) for k in ('x', 'y', 'yaw'))
    return {
        'x': x + math.cos(a) * u - math.sin(a) * v,
        'y': y + math.sin(a) * u + math.cos(a) * v,
        'yaw': math.remainder(a + b, 2 * math.pi),
    }


def nominal_estimate(reference, current, source_pose_in_nominal):
    """Preserve terminal error instead of rebasing nominal P15 onto the actual robot."""
    return compose(source_pose_in_nominal, register_walls(reference, current))


def origin_from_current(current_odom, current_in_nominal):
    """Build a new nominal-to-current-odom transform; never consume historical odom."""
    x, y, a = (finite(current_in_nominal[k]) for k in ('x', 'y', 'yaw'))
    inverse = {
        'x': -math.cos(a) * x - math.sin(a) * y,
        'y': math.sin(a) * x - math.cos(a) * y,
        'yaw': -a,
    }
    return compose(current_odom, inverse)
