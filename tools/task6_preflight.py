"""Read-only live feedback and publisher checks for the Task6 operations wrapper."""

from copy import deepcopy
import math
import time

ODOM = ('/odometry/filtered', '/rosbot_xl_base_controller/odom')
SCAN = '/scan_filtered'


def validate_feedback(samples, now_ros, now_mono, stopped, *, stopped_scan_wait=False):
    if stopped_scan_wait and not stopped:
        raise ValueError('Scan waiting requires stopped feedback')
    scan_fresh = True
    for topic in (*ODOM, SCAN):
        item = samples.get(topic)
        if stopped_scan_wait and topic == SCAN and item is not None:
            age = now_ros - item['stamp']
            if not math.isfinite(age) or item['stamp'] <= 0 or age < -0.1:
                raise RuntimeError(f'Missing/stale source timestamp: {topic}')
            if item['valid_ranges'] < 20 or not item['frame']:
                raise RuntimeError('Insufficient finite scan returns')
            scan_fresh = age <= 0.5 and now_mono - item['receipt'] <= 0.5
            continue
        if item is None or now_mono - item['receipt'] > 0.5:
            raise RuntimeError(f'Missing/stale receipt: {topic}')
        if not -0.1 <= now_ros - item['stamp'] <= 0.5:
            raise RuntimeError(f'Missing/stale source timestamp: {topic}')
        if topic in ODOM:
            if item['frame'] != 'odom' or item['child'] != 'base_link':
                raise RuntimeError(f'Unexpected odometry frames: {topic}')
            if stopped and (
                math.hypot(*item['velocity'][:2]) >= 0.01 or abs(item['velocity'][2]) >= 0.02
            ):
                raise RuntimeError('Robot is not stopped')
        elif item['valid_ranges'] < 20 or not item['frame']:
            raise RuntimeError('Insufficient finite scan returns')
    return scan_fresh


class Probe:
    """Subscriptions only; no speed publisher, initialization or rebase interface."""

    def __init__(self):
        import rclpy
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import LaserScan
        from rclpy.qos import qos_profile_sensor_data
        from rclpy.signals import SignalHandlerOptions
        from rclpy.utilities import get_rmw_implementation_identifier
        from tf2_ros import Buffer, TransformListener

        self.ros = rclpy
        self.rmw_identifier = get_rmw_implementation_identifier()
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        self.node = rclpy.create_node('task6_read_only_preflight')
        self.samples, self.counts, self.failure = {}, {}, None
        self.last_command = None
        self.command_count = 0
        self.last_check = None
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self.node)
        for topic in ODOM:
            self.node.create_subscription(
                Odometry, topic, lambda msg, t=topic: self.odom(t, msg), qos_profile_sensor_data
            )
        self.node.create_subscription(LaserScan, SCAN, self.scan, qos_profile_sensor_data)
        self.node.create_subscription(Twist, '/cmd_vel', self.command, qos_profile_sensor_data)

    def accept(self, topic, state):
        old = self.samples.get(topic)
        if old and state['stamp'] < old['stamp']:
            self.failure = f'Timestamp moved backward: {topic}'
        self.samples[topic] = state
        self.counts[topic] = self.counts.get(topic, 0) + 1

    def odom(self, topic, msg):
        p, q, v = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist
        values = (p.x, p.y, q.x, q.y, q.z, q.w, v.linear.x, v.linear.y, v.angular.z)
        if not all(math.isfinite(n) for n in values):
            self.failure = 'Nonfinite odometry'
            return
        if abs(sum(n * n for n in (q.x, q.y, q.z, q.w)) - 1) > 0.02:
            self.failure = 'Invalid odometry quaternion'
            return
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.accept(
            topic,
            dict(
                stamp=msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9,
                receipt=time.monotonic(),
                frame=msg.header.frame_id,
                child=msg.child_frame_id,
                pose=[p.x, p.y, yaw],
                velocity=[v.linear.x, v.linear.y, v.angular.z],
            ),
        )

    def scan(self, msg):
        self.accept(
            SCAN,
            dict(
                stamp=msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9,
                receipt=time.monotonic(),
                frame=msg.header.frame_id,
                valid_ranges=sum(
                    math.isfinite(v) and msg.range_min <= v <= msg.range_max for v in msg.ranges
                ),
            ),
        )

    def command(self, msg):
        self.command_count += 1
        values = (
            msg.linear.x,
            msg.linear.y,
            msg.linear.z,
            msg.angular.x,
            msg.angular.y,
            msg.angular.z,
        )
        self.last_command = dict(values=values, receipt=time.monotonic())

    def pump(self):
        self.ros.spin_once(self.node, timeout_sec=0.05)

    def identities(self, topic):
        return sorted(
            bytes(i.endpoint_gid).hex() for i in self.node.get_publishers_info_by_topic(topic)
        )

    def endpoints(self, topic):
        """Describe the currently discovered endpoints without spinning or waiting."""
        return sorted(
            (
                dict(gid=bytes(i.endpoint_gid).hex(), node=i.node_name, namespace=i.node_namespace)
                for i in self.node.get_publishers_info_by_topic(topic)
            ),
            key=lambda item: item['gid'],
        )

    def return_context(self):
        """Observe the actual local RMW and rosout graph; never infer identity from logs."""
        observed_at = time.monotonic()
        return dict(
            rmw_identifier=self.rmw_identifier,
            rosout_observed_at=observed_at,
            rosout_endpoints=deepcopy(self.endpoints('/rosout')),
        )

    def diagnostics(self):
        """Capture cached feedback before cleanup; this never validates or permits motion."""
        now_ros = self.node.get_clock().now().nanoseconds / 1e9
        now_mono = time.monotonic()
        return dict(
            unix_time=time.time(),
            ros_time=now_ros,
            monotonic=now_mono,
            topics={
                topic: dict(
                    sample=self.samples.get(topic),
                    count=self.counts.get(topic, 0),
                    source_age_s=now_ros - self.samples[topic]['stamp']
                    if topic in self.samples
                    else None,
                    receipt_age_s=now_mono - self.samples[topic]['receipt']
                    if topic in self.samples
                    else None,
                    publishers=self.endpoints(topic),
                )
                for topic in (*ODOM, SCAN)
            },
            cmd_endpoints=self.endpoints('/cmd_vel'),
            last_command=self.last_command,
            command_count=getattr(self, 'command_count', 0),
            latched_failure=self.failure,
            last_check=getattr(self, 'last_check', None),
        )

    def snapshot(self, stopped=True, expected=None, *, stopped_scan_wait=False):
        self.last_check = dict(
            stopped=stopped,
            cmd_limit=0 if stopped else 1,
            stopped_scan_wait=stopped_scan_wait,
            unix_time=time.time(),
            expected_gids=expected,
        )
        if self.failure:
            raise RuntimeError(self.failure)
        scan_fresh = validate_feedback(
            self.samples,
            self.node.get_clock().now().nanoseconds / 1e9,
            time.monotonic(),
            stopped,
            stopped_scan_wait=stopped_scan_wait,
        )
        gids = {topic: self.identities(topic) for topic in ODOM}
        if any(len(value) != 1 for value in gids.values()) or (
            expected is not None and gids != expected
        ):
            raise RuntimeError('Odometry publisher identity/count changed')
        observed_at = time.monotonic()
        endpoints = self.endpoints('/cmd_vel')
        count = len(endpoints)
        self.last_check.update(
            cmd_count=count, cmd_endpoints=deepcopy(endpoints), cmd_observed_at=observed_at
        )
        if count > (0 if stopped else 1):
            raise RuntimeError(
                'Unexpected cmd_vel publisher; release teleop/controller: '
                f'stopped={stopped} count={count} endpoints={endpoints}'
            )
        transform = self.buffer.lookup_transform(
            'base_link', self.samples[SCAN]['frame'], self.ros.time.Time()
        )
        t, q = transform.transform.translation, transform.transform.rotation
        return dict(
            samples=self.samples.copy(),
            counts=self.counts.copy(),
            scan_fresh=scan_fresh,
            gids=gids,
            cmd_publishers=count,
            cmd_endpoints=deepcopy(endpoints),
            cmd_observed_at=observed_at,
            last_command=self.last_command,
            laser_to_base=dict(translation=[t.x, t.y, t.z], quaternion=[q.x, q.y, q.z, q.w]),
        )

    def ready(self, timeout=8):
        deadline, stable, first, last_error = time.monotonic() + timeout, None, None, 'No feedback'
        while time.monotonic() < deadline:
            self.pump()
            try:
                current = self.snapshot()
                if first is None:
                    first, stable = current, time.monotonic()
                elif current['gids'] != first['gids']:
                    raise RuntimeError('Odometry identity changed during preflight')
                elif time.monotonic() - stable >= 1 and all(
                    current['counts'][t] > first['counts'][t] for t in (*ODOM, SCAN)
                ):
                    return current
            except Exception as error:
                last_error = str(error)
                if self.failure or first is not None:
                    raise
        raise RuntimeError('Preflight unavailable: ' + last_error)

    def wait_zero(self, expected, timeout=8):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.pump()
            if {topic: self.identities(topic) for topic in ODOM} != expected:
                return False
            last = self.last_command
            if (
                not self.identities('/cmd_vel')
                and last is not None
                and all(math.isfinite(v) and abs(v) < 1e-9 for v in last['values'])
            ):
                validate_feedback(
                    self.samples,
                    self.node.get_clock().now().nanoseconds / 1e9,
                    time.monotonic(),
                    True,
                )
                return True
        return False

    def wait_no_command_stop(self, expected, timeout=8):
        """Observe stopped/released feedback without claiming a zero command was received."""
        deadline = time.monotonic() + timeout
        first = None
        while time.monotonic() < deadline:
            self.pump()
            if self.command_count or self.last_command is not None:
                return None
            current = self.snapshot(stopped=True, expected=expected, stopped_scan_wait=True)
            if first is None:
                first = (time.monotonic(), current['counts'])
            if time.monotonic() - first[0] >= 0.3 and all(
                current['counts'][topic] >= first[1][topic] + 3 for topic in ODOM
            ):
                return current
        return None

    def close(self):
        self.node.destroy_node()
        self.ros.shutdown()
