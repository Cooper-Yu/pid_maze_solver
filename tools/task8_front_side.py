"""Explicit single-heading/front plus fixed-reference side-distance pose model.

The right measurement is NOT an independent heading observation. Its normal is
transported from the source right-wall normal using only the front-derived turn.
The caller must establish correspondence to the same physical side wall and
validate along-wall point-cluster overlap externally. Stable fits alone do not
establish that correspondence or remove fixed geometric bias.

No ROS, file access, execution permission, or start-acceptance gate is provided.
The unchanged final initializer must still qualify nominal 2cm/1deg samples.
"""

import math

from task8_p15_localization import compose, normal_wall

MODEL = 'front_heading_fixed_reference_side'


def _wall(side, wall):
    for key in ('count', 'raw_count'):
        value = wall[key]
        if isinstance(value, bool) or not isinstance(value, int) or value < 0:
            raise ValueError('Invalid wall sample counts')
    if wall['count'] > wall['raw_count']:
        raise ValueError('Wall inliers exceed raw count')
    result = normal_wall(side, wall)
    if side == 'front' and (wall['span'] < 0.30 or wall['rms'] > 0.008):
        raise ValueError('Front heading requires span >= .30m and RMS <= .008m')
    return result


def estimate(reference, current, source_pose_in_nominal):
    """Return nominal P15 pose from one front heading and two matched wall offsets.

    Both right-wall distances must be projections onto the fixed source normal:
    source uses ref.right.angle, current uses ref.right.angle minus theta. The
    numeric angle check cannot establish how a distance was actually measured;
    the caller must create the constrained observations from raw points.
    """
    a, b = (_wall(side, reference[side]) for side in ('front', 'right'))
    u, v = (_wall(side, current[side]) for side in ('front', 'right'))
    theta = math.remainder(a[3] - u[3], 2 * math.pi)
    expected = b[3] - theta
    if abs(math.remainder(v[3] - expected, 2 * math.pi)) > 1e-8:
        raise ValueError('Current right distance must use the transported fixed source normal')
    determinant = a[0] * b[1] - a[1] * b[0]
    if abs(determinant) < 0.7:
        raise ValueError('Reference wall normals are too parallel')
    d0, d1 = a[2] - u[2], b[2] - v[2]
    relative = dict(
        x=(d0 * b[1] - a[1] * d1) / determinant, y=(a[0] * d1 - d0 * b[0]) / determinant, yaw=theta
    )
    if math.hypot(relative['x'], relative['y']) > 0.05 or abs(theta) > math.radians(5):
        raise ValueError('Outside bounded fixed-side registration 5cm/5deg region')
    return compose(source_pose_in_nominal, relative)
