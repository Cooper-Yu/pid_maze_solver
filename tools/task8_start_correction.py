"""Pure diagnostic P15 micro-correction planning, never route-start acceptance.

Callers must qualify current independent scan wall fits and fresh stopped feedback.
A returned plan grants no execution permission. After any physical correction, discard
all prior samples and call the unchanged strict P15 initializer again.
"""

import math
from statistics import median

from task8_p15_localization import DIRECTIONS, compose, finite, normal_wall

SCOPE = 'diagnostic_coarse_correction_not_start_acceptance'


def coarse_estimate(reference, current, source_pose_in_nominal, sides=('front', 'right')):
    """Read a bounded coarse two-wall estimate without relaxing strict registration.

    The original wall-quality and nonparallel-line gates are retained. Only this
    diagnostic estimate permits the larger 15 cm / 8 degree region; it is never
    suitable as an accepted route origin.
    """
    if len(sides) != 2 or len(set(sides)) != 2:
        raise ValueError('Two distinct wall directions required')
    for side in sides:
        if side not in DIRECTIONS:
            raise ValueError('Unknown diagnostic wall side')
        for wall in (reference[side], current[side]):
            for key in ('count', 'raw_count'):
                value = wall[key]
                if isinstance(value, bool) or not isinstance(value, int) or value < 0:
                    raise ValueError('Invalid diagnostic wall counts')
            if wall['count'] > wall['raw_count']:
                raise ValueError('Invalid diagnostic wall count relation')
    previous = [normal_wall(side, reference[side]) for side in sides]
    actual = [normal_wall(side, current[side]) for side in sides]
    rotations = [math.remainder(a[3] - b[3], 2 * math.pi) for a, b in zip(previous, actual)]
    disagreement = math.remainder(rotations[0] - rotations[1], 2 * math.pi)
    if abs(disagreement) > math.radians(1.5):
        raise ValueError('Coarse walls disagree on heading')
    theta = math.atan2(sum(math.sin(a) for a in rotations), sum(math.cos(a) for a in rotations))
    a, b = previous
    determinant = a[0] * b[1] - a[1] * b[0]
    if abs(determinant) < 0.7:
        raise ValueError('Coarse wall directions are too parallel')
    d0, d1 = a[2] - actual[0][2], b[2] - actual[1][2]
    relative = dict(
        x=(d0 * b[1] - a[1] * d1) / determinant, y=(a[0] * d1 - d0 * b[0]) / determinant, yaw=theta
    )
    estimate = compose(source_pose_in_nominal, relative)
    _coarse_region(estimate)
    return dict(**estimate, heading_disagreement=disagreement)


def _coarse_region(estimate):
    x, y, yaw = (finite(estimate[key]) for key in ('x', 'y', 'yaw'))
    if math.hypot(x, y) > 0.15 or abs(yaw) > math.radians(8):
        raise ValueError('Outside diagnostic 15cm/8deg correction region')


def _source_sequence(samples, key, distinct):
    stamps = [sample[key] for sample in samples]
    if any(isinstance(v, bool) or not isinstance(v, int) or v <= 0 for v in stamps):
        raise ValueError('Invalid source stamp')
    gaps = [(b - a) / 1e9 for a, b in zip(stamps, stamps[1:])]
    if any(gap < 0 or (distinct and gap == 0) or gap > 0.35 for gap in gaps):
        raise ValueError('Repeated, backward or discontinuous source frames')
    if distinct and (stamps[-1] - stamps[0]) / 1e9 < 1.0:
        raise ValueError('Independent scans must span at least one second')


def _validate(samples):
    if not isinstance(samples, (list, tuple)) or len(samples) < 12:
        raise ValueError('Twelve independent stopped scan estimates required')
    times = [finite(s['time']) for s in samples]
    if times[-1] - times[0] < 1.0 or any(not 0 < b - a <= 0.35 for a, b in zip(times, times[1:])):
        raise ValueError('Observations must span one second with gaps at most .35 seconds')
    _source_sequence(samples, 'scan_stamp', True)
    _source_sequence(samples, 'odom_stamp', False)
    for sample in samples:
        for pose in (sample['odom'], sample['estimate']):
            for key in ('x', 'y', 'yaw'):
                finite(pose[key])
        _coarse_region(sample['estimate'])
        if abs(finite(sample['heading_disagreement'])) > math.radians(1.5):
            raise ValueError('Two-wall heading disagreement exceeds 1.5 degrees')
    for i, first in enumerate(samples):
        for other in samples[i + 1 :]:
            a, b = first['odom'], other['odom']
            if math.hypot(a['x'] - b['x'], a['y'] - b['y']) > 0.005:
                raise ValueError('Odometry position is not stopped within 5 mm')
            if abs(math.remainder(a['yaw'] - b['yaw'], 2 * math.pi)) > 0.01:
                raise ValueError('Odometry heading is not stopped within .01 rad')
    for key, limit in (('x', 0.005), ('y', 0.005), ('yaw', math.radians(1))):
        values = [s['estimate'][key] for s in samples]
        if max(values) - min(values) > limit:
            raise ValueError('Coarse estimate scatter exceeds bounded stability')


def plan(samples):
    """Propose one bounded body-relative micro-action from independent stopped scans.

    All stamps are positive integer nanoseconds; time is monotonically increasing
    receipt time in seconds. Wall quality/freshness checks remain caller duties.
    The result is a diagnostic proposal, not initialization or movement authority.
    """
    try:
        _validate(samples)
    except (KeyError, TypeError) as error:
        raise ValueError('Malformed coarse correction samples') from error
    estimate = {key: median(s['estimate'][key] for s in samples) for key in ('x', 'y', 'yaw')}
    x, y, yaw = (estimate[k] for k in ('x', 'y', 'yaw'))
    result = dict(
        action='wait',
        angle_rad=0.0,
        forward_m=0.0,
        left_m=0.0,
        median_estimate=estimate,
        sample_count=len(samples),
        scope=SCOPE,
        motion_authorized=False,
    )
    if abs(yaw) > math.radians(0.5):
        result['action'] = 'turn'
        # Below the follower's .01 rad arrival tolerance, half-gain would request no motion.
        gain = 1.0 if abs(yaw) < 0.02 else 0.5
        result['angle_rad'] = max(-math.radians(2), min(math.radians(2), -gain * yaw))
    elif math.hypot(x, y) > 0.008:
        forward, left = (
            -math.cos(yaw) * x - math.sin(yaw) * y,
            math.sin(yaw) * x - math.cos(yaw) * y,
        )
        length = math.hypot(forward, left)
        scale = min(1.0, 0.03 / length)
        result.update(action='translate', forward_m=forward * scale, left_m=left * scale)
    return result
