"""Pure bounded wall-pose servo command, without ROS or execution authority.

Readiness is instantaneous only. Runtime must validate wall measurement quality,
maintain unique control ownership, and zero/release in finally on every exit.
An independent strict initializer must still accept the eventual route start.
"""

import math


def _finite(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError('Wall servo inputs must be finite numbers')
    return float(value)


def velocity(estimate, drift_m, elapsed_s, source_age_s, receipt_age_s):
    """Return ((vx, vy, wz), ready) from nominal-frame wall pose, meters/radians.

    Translation is transformed into the current body frame. Heading feedback
    comes directly from the validated wall estimate, without an odom turn target
    or the Console .01 rad arrival deadband. Invalid/stale/out-of-budget data
    raise ValueError; this pure function cannot send the required stop itself.
    """
    try:
        x, y, yaw = (_finite(estimate[key]) for key in ('x', 'y', 'yaw'))
    except (KeyError, TypeError) as error:
        raise ValueError('Wall servo requires x/y/yaw pose estimate') from error
    drift, elapsed, source_age, receipt_age = (
        _finite(value) for value in (drift_m, elapsed_s, source_age_s, receipt_age_s)
    )
    if not -0.1 <= source_age <= 0.30 or not 0 <= receipt_age <= 0.30:
        raise ValueError('Wall servo source or receipt feedback stale/invalid')
    if not 0 <= elapsed <= 30:
        raise ValueError('Wall servo 30 second budget exceeded/invalid')
    if not 0 <= drift <= 0.04:
        raise ValueError('Wall servo 4 cm drift budget exceeded/invalid')
    position = math.hypot(x, y)
    if position > 0.05 or abs(yaw) > math.radians(8):
        raise ValueError('Wall servo estimate outside 5cm/8deg region')
    heading_ready = abs(yaw) <= math.radians(0.3)
    ready = position <= 0.005 and heading_ready
    wz = 0.0 if heading_ready else max(-0.02, min(0.02, -0.5 * yaw))
    vx = vy = 0.0
    if position > 0.005 and abs(yaw) <= math.radians(0.8):
        c, s = math.cos(yaw), math.sin(yaw)
        vx, vy = -0.5 * (c * x + s * y), 0.5 * (s * x - c * y)
        scale = min(1.0, 0.01 / math.hypot(vx, vy))
        vx, vy = vx * scale, vy * scale
    return (vx, vy, wz), ready
