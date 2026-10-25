"""Coarse side-wall heading acquisition; never a position or route-start acceptance."""

from dataclasses import asdict, replace
import math
from statistics import median


def fit_side(points, counts, side):
    """Keep real +/-20deg rays and coverage; only lift the coarse TLS angle domain."""
    from wall_geometry import fit_wall, support

    reason = 'No reliable coarse side wall'
    for offset in (0.0, math.radians(20), math.radians(-20)):
        c, s = math.cos(offset), math.sin(offset)
        rotated = [(c * x + s * y, -s * x + c * y, a) for x, y, a in points]
        try:
            wall = fit_wall(rotated, counts, side)
            angle = math.remainder(wall.angle + offset, 2 * math.pi)
            if abs(angle) > math.radians(30):
                raise ValueError('Coarse side wall outside 30deg observation domain')
            center = math.pi / 2 if side == 'left' else -math.pi / 2
            distance = wall.gap + support(
                math.cos(center + wall.angle), math.sin(center + wall.angle)
            )
            gap = distance - support(math.cos(center + angle), math.sin(center + angle))
            if gap <= 0:
                raise ValueError('Coarse side wall intersects body support')
            return asdict(replace(wall, angle=angle, gap=gap))
        except ValueError as error:
            reason = str(error)
    raise ValueError(reason)


def observe(points, counts, reference, source_yaw):
    """Two observed sides corroborate direction; recorded right wall anchors heading."""
    from wall_geometry import support

    sides = {side: fit_side(points, counts, side) for side in ('left', 'right')}
    left, right = sides['left'], sides['right']
    if abs(math.remainder(left['angle'] - right['angle'], 2 * math.pi)) > math.radians(4):
        raise ValueError('Coarse side walls disagree by more than 4deg')
    old = reference['right']

    def offset(wall):
        a = -math.pi / 2 + wall['angle']
        return wall['gap'] + support(math.cos(a), math.sin(a))

    if abs(offset(right) - offset(old)) > 0.10:
        raise ValueError('Coarse right wall outside 10cm reference neighborhood')
    correction = math.remainder(right['angle'] - old['angle'] - source_yaw, 2 * math.pi)
    if not math.isfinite(correction) or abs(correction) > math.radians(25):
        raise ValueError('Coarse heading correction outside 25deg acquisition region')
    return dict(correction=correction, walls=sides)


def qualify(samples):
    """Fresh, stationary twelve-frame batch; no nominal position is invented."""
    from task8_p15_localization import require_source_sequences

    if len(samples) != 12 or samples[-1]['time'] - samples[0]['time'] < 1:
        raise ValueError('Coarse heading needs twelve frames spanning one second')
    require_source_sequences(samples)
    if any(not 0 < b['time'] - a['time'] <= 0.35 for a, b in zip(samples, samples[1:])):
        raise ValueError('Coarse heading observations not consecutive')
    first = samples[0]['odom']
    angles = []
    for sample in samples:
        p = sample['odom']
        if not all(math.isfinite(v) for v in (*p.values(), sample['correction'])):
            raise ValueError('Nonfinite coarse heading observation')
        if (
            math.hypot(p['x'] - first['x'], p['y'] - first['y']) > 0.005
            or abs(math.remainder(p['yaw'] - first['yaw'], 2 * math.pi)) > 0.01
        ):
            raise ValueError('Robot moved during coarse heading acquisition')
        angles.append(sample['correction'])
    if max(angles) - min(angles) > math.radians(1) or abs(
        median(angles[:6]) - median(angles[6:])
    ) > math.radians(0.5):
        raise ValueError('Coarse heading batches unstable')
    return median(angles)
