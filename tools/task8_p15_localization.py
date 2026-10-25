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


def require_source_sequences(samples):
    """Reject replayed or backward source stamps before considering pose estimates."""
    for key in ('scan_stamp', 'odom_stamp'):
        stamps = [s[key] for s in samples]
        if any(isinstance(v, bool) or not isinstance(v, int) or v <= 0 for v in stamps):
            raise ValueError('P15 invalid source timestamp')
        if any(b < a for a, b in zip(stamps, stamps[1:])):
            raise ValueError('P15 source clock moved backward')
        required = len(samples) if key == 'scan_stamp' else 10
        if len(set(stamps)) < required:
            raise ValueError('P15 repeated source frames')


def qualify_samples(samples):
    """Precise aligned start; unchanged default 2cm/1deg contract."""
    return _qualify_stopped(samples, 0.02, math.radians(1), '2cm/1deg')


def qualify_handoff_samples(samples):
    """Located near P15, not aligned: preserve offset for the existing first path."""
    return _qualify_stopped(samples, 0.05, math.radians(8), '5cm/8deg localized handoff')


def _qualify_stopped(samples, position_limit, yaw_limit, label):
    """Require two agreeing stopped batches and every estimate inside the 2cm/1deg gate."""
    from statistics import median

    if len(samples) < 12 or samples[-1]['time'] - samples[0]['time'] < 1.0:
        raise ValueError('P15 requires twelve samples spanning one second')
    require_source_sequences(samples)
    times = [finite(s['time']) for s in samples]
    if any(not 0 < b - a <= 0.35 for a, b in zip(times, times[1:])):
        raise ValueError('P15 samples are not consecutive fresh observations')
    first = samples[0]['odom']
    for sample in samples:
        pose, estimate = sample['odom'], sample['estimate']
        for values in (pose, estimate):
            for key in ('x', 'y', 'yaw'):
                finite(values[key])
        if math.hypot(pose['x'] - first['x'], pose['y'] - first['y']) > 0.005:
            raise ValueError('P15 moved during initialization')
        if abs(math.remainder(pose['yaw'] - first['yaw'], 2 * math.pi)) > 0.01:
            raise ValueError('P15 turned during initialization')
        if (
            math.hypot(estimate['x'], estimate['y']) > position_limit
            or abs(estimate['yaw']) > yaw_limit
        ):
            raise ValueError('P15 estimate outside nominal ' + label + ' gate')
    half = len(samples) // 2
    centers = [
        {k: median(s['estimate'][k] for s in group) for k in ('x', 'y', 'yaw')}
        for group in (samples[:half], samples[half:])
    ]
    if math.hypot(centers[0]['x'] - centers[1]['x'], centers[0]['y'] - centers[1]['y']) > 0.005:
        raise ValueError('P15 position batches disagree')
    if abs(centers[0]['yaw'] - centers[1]['yaw']) > math.radians(0.5):
        raise ValueError('P15 heading batches disagree')
    return {k: median(s['estimate'][k] for s in samples) for k in ('x', 'y', 'yaw')}
